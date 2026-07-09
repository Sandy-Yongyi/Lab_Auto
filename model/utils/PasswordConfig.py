import tomllib


def is_password_required(mode_config_path: str) -> bool:
    """读取 ModeConfig.toml 的 require_password，读取失败时默认需要密码。"""
    try:
        with open(mode_config_path, "rb") as f:
            config = tomllib.load(f)
    except Exception:
        return True

    value = config.get("require_password", True)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in {"false", "0", "no", "off"}
    return bool(value)
