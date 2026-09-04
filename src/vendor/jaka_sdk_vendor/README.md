# jaka_sdk_vendor

JAKA C API 的唯一仓库内来源。驱动工具和 `ros2_control` 硬件插件必须通过
`find_package(jaka_sdk_vendor)` 使用这里的头文件与 `libjakaAPI.so`，不得再复制 SDK。

该二进制 SDK 的再分发和版本信息应在获得厂商正式材料后补充；当前仓库历史没有记录
其精确厂商版本。
