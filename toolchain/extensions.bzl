load(":repo_rules.bzl", "arm_none_eabi_toolchain_repo")

def _arm_none_eabi_impl(ctx):
    arm_none_eabi_toolchain_repo(name = "arm_none_eabi")

arm_none_eabi_ext = module_extension(
    implementation = _arm_none_eabi_impl,
)