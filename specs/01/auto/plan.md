# Auto Extension Implementation Plan

## 1. Create Extension Structure
- Create `deps_rocker/extensions/auto/` directory
- Create `__init__.py` with proper imports
- Create `auto.py` with main extension class

## 2. Implement Auto Class
The `Auto` class should:
- Inherit from `SimpleRockerExtension`
- Set `name = "auto"`
- Override `required()` method to perform file detection
- Implement file pattern detection logic:
  - Use `os.path.exists()` for exact file matches
  - Use `glob.glob()` for pattern matching (e.g., `*.cpp`)
  - Check current working directory recursively for patterns
- Map detected patterns to extension dependencies
- Return set of extension names based on detected files

## 3. File Detection Mapping
Create a method that checks for:
- `pixi.toml` → add "pixi"
- `pyproject.toml` OR `requirements*.txt` OR `.python-version` OR `poetry.lock` → add "uv"
- `package.json` → add "npm"
- `Cargo.toml` → add "cargo"
- `environment.yml` OR `environment.yaml` → add "conda"
- `package.xml` → add "ros_jazzy"
- Any `.cpp`, `.hpp`, `.cc`, `.cxx`, `.h`, `.c` files → add "ccache"

## 4. Add Entry Point
Update `pyproject.toml` to register the auto extension:
```toml
auto = "deps_rocker.extensions.auto.auto:Auto"
```

## 5. Testing
- Add `auto` to `EXTENSIONS_TO_TEST` in `test/test_extensions_generic.py`
- Create test method to verify file detection works
- Test that it doesn't error when no files are detected
- Create integration test that creates test files and verifies correct dependencies are returned

## 6. No Dockerfile Snippet Needed
This extension doesn't install anything - it just orchestrates other extensions, so no `auto_snippet.Dockerfile` is required.

## 7. Edge Cases to Consider
- Empty directories (no files detected) - should not error, return empty set
- Multiple patterns detected - should return union of all matched extensions
- Nested directories - decide whether to search recursively or just top-level
- Performance - don't scan too deeply, maybe limit depth or use caching
