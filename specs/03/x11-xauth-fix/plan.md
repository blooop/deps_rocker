# Implementation Plan: X11 Xauth Fix

## Phase 1: Documentation (Immediate)

### 1. Update README.md
Add troubleshooting section for X11 extension:

```markdown
## Troubleshooting

### X11 Extension: `.Xauthority does not exist` Error

If you see this error:
```
xauth:  file /home/user/.Xauthority does not exist
xauth: (argv):1:  unable to read any entries from file "(stdin)"
```

**Quick Fix:**
```bash
touch ~/.Xauthority
```

**Alternative:** If you don't need GUI support, remove `x11` from your `rockerc.yaml` args list.
```

### 2. Test the workaround
- Verify `touch ~/.Xauthority` resolves the issue
- Document any edge cases

## Phase 2: Wrapper Extension (If Needed)

### 1. Create `x11_safe` extension
```python
# deps_rocker/extensions/x11_safe/x11_safe.py
class X11Safe(SimpleRockerExtension):
    """Safely enable X11 by ensuring .Xauthority exists"""

    name = "x11_safe"
    depends_on_extension = ("x11",)

    def invoke_before(self, args):
        """Create .Xauthority if it doesn't exist before X11 extension runs"""
        # Check and create ~/.Xauthority if missing
        # Return appropriate docker RUN commands
```

### 2. Add entry point to pyproject.toml

### 3. Create tests
- Test with missing .Xauthority
- Test with existing .Xauthority
- Verify X11 extension still works

### 4. Update documentation
- Add x11_safe to available extensions list
- Explain when to use x11_safe vs x11

## Decision Point
**Start with Phase 1 only.** Implement Phase 2 only if users request it or if the workaround proves insufficient.

## Testing Strategy
1. Remove ~/.Xauthority and test rocker with x11 extension
2. Apply fix (touch ~/.Xauthority)
3. Verify X11 forwarding works
4. Document results
