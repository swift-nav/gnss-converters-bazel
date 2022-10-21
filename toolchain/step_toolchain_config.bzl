load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "feature", "flag_group", "flag_set", "tool_path")
load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")

SDK_PATH_PREFIX = "/opt/poky-st/2.6/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-{}"

def _impl(ctx):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = SDK_PATH_PREFIX.format("gcc"),
        ),
        tool_path(
            name = "ld",
            path = SDK_PATH_PREFIX.format("ld"),
        ),
        tool_path(
            name = "ar",
            path = SDK_PATH_PREFIX.format("ar"),
        ),
        tool_path(
            name = "cpp",
            path = SDK_PATH_PREFIX.format("cpp"),
        ),
        tool_path(
            name = "gcov",
            path = SDK_PATH_PREFIX.format("gcov"),
        ),
        tool_path(
            name = "nm",
            path = SDK_PATH_PREFIX.format("nm"),
        ),
        tool_path(
            name = "objdump",
            path = SDK_PATH_PREFIX.format("objdump"),
        ),
        tool_path(
            name = "strip",
            path = SDK_PATH_PREFIX.format("strip"),
        ),
    ]

    features = [
        feature(
            name = "default_cc_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile
                    ],
                    flag_groups = ([
                        flag_group(
                            flags = [
                                "-march=armv7ve",
                                "-mthumb",
                                "-mfpu=neon",
                                "-mfloat-abi=hard",
                                "-mcpu=cortex-a7",
                                "-O2",
                                "-pipe",
                                "-g",
                                "-feliminate-unused-debug-types",
                            ],
                        ),
                    ]),
                ),
            ],
        ),
        feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.cpp_link_executable,
                        ACTION_NAMES.cpp_link_dynamic_library,
                        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                    ],
                    flag_groups = ([
                        flag_group(
                            flags = [
                                "-lm",
                                "-lstdc++",
                                "-mfloat-abi=hard",
                                "-march=armv7ve",
                                "-mthumb",
                                "-mfpu=neon",
                                "-mcpu=cortex-a7",
                                "-Wl,-O1",
                                "-Wl,--hash-style=gnu",
                                "-Wl,--as-needed",
                            ],
                        ),
                    ]),
                ),
            ],
        ),
    ]

    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        features = features,
        cxx_builtin_include_directories = [
            "/opt/poky-st/2.6/sysroots/cortexa7t2hf-neon-poky-linux-gnueabi/usr/include",
            "/opt/poky-st/2.6/sysroots/x86_64-pokysdk-linux/usr/lib/arm-poky-linux-gnueabi/gcc/arm-poky-linux-gnueabi/8.2.0/include",
            "/opt/poky-st/2.6/sysroots/x86_64-pokysdk-linux/usr/lib/arm-poky-linux-gnueabi/gcc/arm-poky-linux-gnueabi/8.2.0/include-fixed",
        ],
        builtin_sysroot = "/opt/poky-st/2.6/sysroots/cortexa7t2hf-neon-poky-linux-gnueabi",
        toolchain_identifier = "step-toolchain",
        host_system_name = "local",
        target_system_name = "local",
        target_cpu = "step",
        target_libc = "unknown",
        compiler = "gcc",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = tool_paths,
    )

step_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)
