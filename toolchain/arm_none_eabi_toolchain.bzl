load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "action_config",
    "feature",
    "flag_group",
    "flag_set",
    "tool",
    "tool_path",
    "variable_with_value"
)
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/toolchains:cc_toolchain.bzl", "cc_toolchain")

# load(
#     "@rules_cc//cc/common:cc_common.bzl",
#     "CcToolchainConfigInfo"
# )

def _arm_none_eabi_toolchain_config_impl(ctx):
    bin_prefix  = ctx.attr.bin_prefix
    toolchain_dir = ctx.attr.toolchain_dir
    gcc_version = ctx.attr.gcc_version
    cpu_flags = ctx.attr.cpu_flags

    tool_paths = [
        tool_path(name = "gcc", path = bin_prefix + "-gcc"),
        tool_path(name = "g++", path = bin_prefix + "-g++"),
        tool_path(name = "ar", path = bin_prefix + "-ar"),
        tool_path(name = "ld", path = bin_prefix + "-ld"),
        tool_path(name = "nm", path = bin_prefix + "-nm"),
        tool_path(name = "objdump", path = bin_prefix + "-objdump"),
        tool_path(name = "objcopy", path = bin_prefix + "-objcopy"),
        tool_path(name = "strip", path = bin_prefix + "-strip"),
        tool_path(name = "gcov", path = bin_prefix + "-gcov"),
        # Bazel requires these even if unused
        tool_path(name = "cpp", path = "/usr/bin/cpp"),
        tool_path(name = "dwp", path = "/usr/bin/dwp"),
        tool_path(name = "llvm-cov", path = "/usr/bin/false"),
    ]

    builtin_include_dirs = [
        toolchain_dir + "/lib/gcc/arm-none-eabi/" + gcc_version + "/include",
        toolchain_dir + "/lib/gcc/arm-none-eabi/" + gcc_version + "/include-fixed",
        toolchain_dir + "/arm-none-eabi/include"
    ]

        # Action configs wire action names to the actual tool binaries
    action_configs = [
        action_config(
            action_name = ACTION_NAMES.c_compile,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.cpp_compile,
            enabled = True,
            tools = [tool(path = bin_prefix + "-g++")],
        ),
        action_config(
            action_name = ACTION_NAMES.assemble,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.preprocess_assemble,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.cpp_link_executable,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.cpp_link_static_library,
            enabled = True,
            tools = [tool(path = bin_prefix + "-ar")],
        ),
        action_config(
            action_name = ACTION_NAMES.cpp_link_dynamic_library,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.cpp_link_nodeps_dynamic_library,
            enabled = True,
            tools = [tool(path = bin_prefix + "-gcc")],
        ),
        action_config(
            action_name = ACTION_NAMES.strip,
            enabled = True,
            tools = [tool(path = bin_prefix + "-strip")],
        ),
    ]

    compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
    ]

    link_actions = [
        ACTION_NAMES.cpp_link_executable,
        ACTION_NAMES.cpp_link_dynamic_library,
        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
    ]

    common_flags = cpu_flags + [
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-exceptions",
        "-fno-unwind-tables",
        "-Os",
    ]

    complie_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
    ]

    link_actions = [
        ACTION_NAMES.cpp_link_executable,
        ACTION_NAMES.cpp_link_dynamic_library,
        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
    ]

    features = [
        feature(
            name = "archiver_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [ACTION_NAMES.cpp_link_static_library],
                    flag_groups = [
                        flag_group(
                            flags = ["rcsD"],
                        ),
                        flag_group(
                            flags = ["%{output_execpath}"],
                            expand_if_available = "output_execpath",
                        ),
                    ],
                ),
                flag_set(
                    actions = [ACTION_NAMES.cpp_link_static_library],
                    flag_groups = [
                        flag_group(
                            iterate_over = "libraries_to_link",
                            flag_groups = [
                                flag_group(
                                    flags = ["%{libraries_to_link.name}"],
                                    expand_if_equal = variable_with_value(
                                        name = "libraries_to_link.type",
                                        value = "object_file",
                                    ),
                                ),
                            ],
                            expand_if_available = "libraries_to_link",
                        ),
                    ],
                ),
            ],
        ),
        feature(
            name = "default_complie_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = complie_actions,
                    flag_groups = [flag_group(flags = common_flags)],
                ),
            ],
        ),
        feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = link_actions,
                    flag_groups = [flag_group(flags = cpu_flags + [
                        "-Wl,--gc-sections",
                        "-specs=nosys.specs",
                        "-specs=nano.specs",
                        "-lstdc++",
                        "-lsupc++",
                        "-lc",
                        "-lgcc",
                        "-lm",
                        "-lnosys",
                    ])],
                ),
            ],
        ),
        feature(name = "supports_pic", enabled = False),
        feature(name = "supports_dynamic_linker", enabled = False),
        # feature(name = "no_legacy_features"),
    ]

    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        action_configs = action_configs,
        features = features,
        cxx_builtin_include_directories = builtin_include_dirs,
        toolchain_identifier = "arm-none-eabi-stm32",
        host_system_name = "local",
        target_system_name = "arm-none-eabi",
        target_cpu = "cortex-m4",
        target_libc = "newlib",
        compiler = "gcc",
        abi_version = "eabi",
        abi_libc_version = "newlib",
        tool_paths = tool_paths,
    )

_arm_none_eabi_toolchain_config = rule(
    implementation = _arm_none_eabi_toolchain_config_impl,
    attrs = {
        "bin_prefix": attr.string(mandatory = True),
        "toolchain_dir": attr.string(mandatory = True),
        "gcc_version": attr.string(mandatory = True),
        "cpu_flags": attr.string_list(default = []),
    },
)

def arm_none_eabi_toolchain(name, bin_prefix, toolchain_dir, gcc_version, toolchain_files, cpu_flags):
    config_name = name + "_config"

    _arm_none_eabi_toolchain_config(
        name = config_name,
        bin_prefix = bin_prefix,
        toolchain_dir = toolchain_dir,
        gcc_version = gcc_version,
        cpu_flags = cpu_flags,
    )

    cc_toolchain(
        name = name,
        toolchain_config = ":" + config_name,
        all_files = toolchain_files,
        compiler_files = toolchain_files,
        linker_files = toolchain_files,
        dwp_files = ":empty",
        objcopy_files = toolchain_files,
        strip_files = toolchain_files,
        supports_param_files = False,
    )

    native.filegroup(name = "empty", srcs = [])
