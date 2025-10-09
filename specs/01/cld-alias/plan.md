# Implementation Plan: cld alias

## Steps

1. **Modify claude_user_snippet.Dockerfile**
   - After installing claude, append the alias to `.bashrc`
   - Also append to `.zshrc` if the file exists
   - Use a comment to identify the alias for clarity

2. **Test**
   - Run `pixi run ci` to ensure tests pass
   - Verify the alias is present in the shell config files

3. **Commit**
   - Commit spec directory first
   - Then commit the implementation changes after ci passes

## Technical Details

The alias will be added using:
```bash
echo 'alias cld="claude --dangerously-skip-permissions"' >> ~/.bashrc
```

And similarly for zsh if it exists.
