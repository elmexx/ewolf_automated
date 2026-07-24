"""Small YAML compatibility wrapper with a minimal fallback parser."""
import json

try:  # pragma: no cover
    import yaml as _yaml
except ModuleNotFoundError:  # pragma: no cover - exercised when PyYAML absent
    _yaml = None


def safe_load(stream):
    text = stream.read() if hasattr(stream, "read") else str(stream)
    if _yaml is not None:
        return _yaml.safe_load(text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return _parse_simple_yaml(text)


def safe_dump(data, stream=None, sort_keys=True):
    if _yaml is not None:
        return _yaml.safe_dump(data, stream, sort_keys=sort_keys)
    text = json.dumps(data, indent=2, sort_keys=sort_keys)
    if stream is not None:
        stream.write(text)
        return None
    return text


def _parse_scalar(value):
    if value in ("", None):
        return {}
    value = value.strip()
    if value in ("true", "True"):
        return True
    if value in ("false", "False"):
        return False
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value.strip('"\'')


def _parse_simple_yaml(text):
    root = {}
    stack = [(-1, root)]
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        line = raw.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if line.startswith("- "):
            item = line[2:]
            if not isinstance(parent, list):
                raise ValueError("minimal YAML parser found list item without list parent")
            if ":" in item:
                key, value = item.split(":", 1)
                obj = {key.strip(): _parse_scalar(value.strip())}
                parent.append(obj)
                if value.strip() == "":
                    stack.append((indent, obj[key.strip()]))
                else:
                    stack.append((indent, obj))
            else:
                parent.append(_parse_scalar(item))
            continue
        key, value = line.split(":", 1)
        key = key.strip(); value = value.strip()
        if value == "":
            # Look ahead using indentation is intentionally simple: common config
            # list-valued keys are named topics/core_domains/etc.
            container = [] if key in {"topics", "core_domains", "optional_domains", "full_requires", "requires"} else {}
            parent[key] = container
            stack.append((indent, container))
        else:
            parent[key] = _parse_scalar(value)
    return root

YAMLError = ValueError
