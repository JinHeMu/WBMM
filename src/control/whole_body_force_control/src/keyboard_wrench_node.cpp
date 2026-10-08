#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <X11/Xlib.h>
#include <X11/XKBlib.h>
#include <X11/keysym.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Sends commands to the single virtual sensor, never to a hardware interface
// or the processed-wrench topic. X11 supplies actual key-down/key-up events.
class KeyboardWrenchNode final : public rclcpp::Node {
public:
  KeyboardWrenchNode() : Node("keyboard_force_publisher") {
    const auto topic = declare_parameter<std::string>(
        "command_topic", "/whole_body_force_control/virtual_wrench_command");
    frame_ = declare_parameter<std::string>("frame_id", "tool0");
    const auto force = declare_parameter<std::vector<double>>("force_n", {5., 5., 5.});
    const auto keys = declare_parameter<std::vector<std::string>>(
        "keys", {"w", "s", "a", "d", "r", "f"});
    const double rate = declare_parameter<double>("rate", 50.0);
    if (topic.empty() || frame_.empty() || force.size() != 3 || keys.size() != 6 ||
        !std::isfinite(rate) || rate < 10.0 || rate > 1000.0) {
      throw std::invalid_argument("invalid keyboard force configuration");
    }
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(force[i]) || force[i] < 0.0) {
        throw std::invalid_argument("force_n requires three finite nonnegative values");
      }
      force_[i] = force[i];
    }
    for (size_t i = 0; i < 6; ++i) {
      symbols_[i] = XStringToKeysym(keys[i].c_str());
      if (symbols_[i] == NoSymbol || symbols_[i] == XK_space || symbols_[i] == XK_Escape) {
        throw std::invalid_argument("keys must contain six distinct usable X11 key names");
      }
      for (size_t j = 0; j < i; ++j) {
        if (symbols_[j] == symbols_[i]) { throw std::invalid_argument("duplicate keyboard key"); }
      }
      labels_[i] = keys[i];
    }
    display_ = XOpenDisplay(nullptr);
    if (!display_) { throw std::runtime_error("Cannot open DISPLAY; run from the graphical desktop terminal"); }
    window_ = XCreateSimpleWindow(display_, DefaultRootWindow(display_), 80, 80, 610, 255,
                                 1, BlackPixel(display_, 0), WhitePixel(display_, 0));
    XStoreName(display_, window_, "WBMM Virtual Force - hold keys, release to zero");
    XSelectInput(display_, window_, KeyPressMask | KeyReleaseMask | ExposureMask | FocusChangeMask);
    close_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(display_, window_, &close_, 1);
    Bool supported = False;
    XkbSetDetectableAutoRepeat(display_, True, &supported);
    detectableRepeat_ = supported;
    gc_ = XCreateGC(display_, window_, 0, nullptr);
    XMapWindow(display_, window_);
    XFlush(display_);
    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic, 1);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Keyboard force window: %s (+X/-X), %s (+Y/-Y), %s (+Z/-Z); frame=%s topic=%s",
                (labels_[0] + "/" + labels_[1]).c_str(), (labels_[2] + "/" + labels_[3]).c_str(),
                (labels_[4] + "/" + labels_[5]).c_str(), frame_.c_str(), topic.c_str());
  }

  ~KeyboardWrenchNode() override {
    if (display_) {
      XFreeGC(display_, gc_);
      XDestroyWindow(display_, window_);
      XCloseDisplay(display_);
    }
  }

private:
  bool physicalForceKeyHeld() {
    char physical[32];
    XQueryKeymap(display_, physical);
    for (const auto symbol : symbols_) {
      const auto code = XKeysymToKeycode(display_, symbol);
      if (physical[code / 8] & (1 << (code % 8))) { return true; }
    }
    return false;
  }

  void draw() {
    XClearWindow(display_, window_);
    auto line = [this](int y, const std::string &text) {
      XDrawString(display_, window_, gc_, 20, y, text.c_str(), static_cast<int>(text.size()));
    };
    line(30, "Wait for ACTIVE, click here, HOLD a key. Release = zero.");
    line(58, "Force axes: " + frame_ + " | torque = zero | simultaneous axes supported");
    for (size_t axis = 0; axis < 3; ++axis) {
      char text[180];
      std::snprintf(text, sizeof(text), "%c:  %s = +%.2f N     %s = -%.2f N     current: %+.2f N",
                    "XYZ"[axis], labels_[2 * axis].c_str(), force_[axis],
                    labels_[2 * axis + 1].c_str(), force_[axis], current_[axis]);
      line(96 + static_cast<int>(axis) * 31, text);
    }
    line(207, "SPACE: clear force (release and press again)    ESC / close: exit");
    line(233, "Losing window focus clears all forces.");
    XFlush(display_);
  }

  void tick() {
    bool redraw = false, quit = false;
    while (XPending(display_)) {
      XEvent event;
      XNextEvent(display_, &event);
      if (event.type == Expose) { redraw = true; }
      else if (event.type == FocusOut) { held_.fill(false); inhibited_ = true; redraw = true; }
      else if (event.type == FocusIn) { inhibited_ = physicalForceKeyHeld(); }
      else if (event.type == ClientMessage && static_cast<Atom>(event.xclient.data.l[0]) == close_) { quit = true; }
      else if (event.type == KeyPress || event.type == KeyRelease) {
        // Legacy X11 auto-repeat emits release/press pairs with identical time.
        if (event.type == KeyRelease && !detectableRepeat_ && XPending(display_)) {
          XEvent next;
          XPeekEvent(display_, &next);
          if (next.type == KeyPress && next.xkey.time == event.xkey.time &&
              next.xkey.keycode == event.xkey.keycode) { continue; }
        }
        const KeySym key = XLookupKeysym(&event.xkey, 0);
        if (key == XK_Escape) { quit = true; }
        if (key == XK_space && event.type == KeyPress) { held_.fill(false); inhibited_ = true; }
        if (inhibited_) {
          // Do not re-arm from the repeat stream after SPACE/focus changes.
          if (!physicalForceKeyHeld()) { inhibited_ = false; }
        } else {
          for (size_t i = 0; i < 6; ++i) {
            if (key == symbols_[i]) { held_[i] = event.type == KeyPress; }
          }
        }
        redraw = true;
      }
    }
    if (quit) { held_.fill(false); }
    geometry_msgs::msg::WrenchStamped message;
    message.header.frame_id = frame_;
    for (size_t i = 0; i < 3; ++i) {
      current_[i] = force_[i] * (static_cast<int>(held_[2 * i]) - static_cast<int>(held_[2 * i + 1]));
    }
    message.wrench.force.x = current_[0]; message.wrench.force.y = current_[1];
    message.wrench.force.z = current_[2];
    publisher_->publish(message);
    if (redraw) { draw(); }
    if (quit) { rclcpp::shutdown(); }
  }

  std::string frame_;
  std::array<double, 3> force_{}, current_{};
  std::array<KeySym, 6> symbols_{};
  std::array<std::string, 6> labels_{};
  std::array<bool, 6> held_{};
  Display *display_{nullptr};
  Window window_{};
  GC gc_{};
  Atom close_{};
  bool detectableRepeat_{false}, inhibited_{false};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<KeyboardWrenchNode>()); }
  catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("keyboard_force_publisher"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
