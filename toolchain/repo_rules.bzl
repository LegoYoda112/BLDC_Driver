load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

_TOOLCHAIN_VERSION = "15.2.rel1"

_TOOLCHAIN_URLS = {
    "macos-aarch64": {
        "urls": ["https://developer.arm.com/-/media/Files/downloads/gnu/{}/binrel/arm-gnu-toolchain-{}-darwin-arm64-arm-none-eabi.tar.xz".format(_TOOLCHAIN_VERSION, _TOOLCHAIN_VERSION)],
        "sha256": "1938a84b7105c192e3fb4fa5e893ba25f425f7ddab40515ae608cd40f68669a8",
        "prefix": "arm-gnu-toolchain-{}-darwin-arm64-arm-none-eabi/bin/arm-none-eabi".format(_TOOLCHAIN_VERSION)
    }
}

def _arm_none_eabi_toolchain_repo_impl(ctx):
    os_name = ctx.os.name.lower()
    arch = ctx.os.arch.lower()

    if "mac" in os_name or "darwin" in os_name:
        key = "macos-aarch64" if "aarch64" in arch or "arm" in arch else "macos-x86_64"
    else:
        fail("OS {} is not supported yet by the toolchain".format(os_name, arch))

    spec = _TOOLCHAIN_URLS[key]

    ctx.download_and_extract(
        url = spec["urls"],
        sha256 = spec["sha256"]
    )

    repo_root = str(ctx.path("."))

    toolchain_dir = spec["prefix"][:spec["prefix"].rfind("/bin/")]

    result = ctx.execute(["bash", "-c", "ls {}/{}/lib/gcc/arm-none-eabi/".format(repo_root, toolchain_dir)])
    gcc_version = result.stdout.strip()


    ctx.file("defs.bzl", content = """

TOOLCHAIN_DIR = "{repo_root}/{toolchain_dir}"
BIN_PREFIX = "{repo_root}/{prefix}"
GCC_VERSION = "{gcc_version}"
""".format(toolchain_dir = toolchain_dir,
            repo_root = repo_root,
            prefix = spec["prefix"],
            gcc_version = gcc_version)
    )

    ctx.file("BUILD", content = """
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "all_files",
    srcs = glob(["**/*"])
)"""
    )

arm_none_eabi_toolchain_repo = repository_rule(
    implementation = _arm_none_eabi_toolchain_repo_impl,
    attrs = {},
)