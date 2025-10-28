# Plan: Fix Auto Extension Search Root

1. Update spec.md with requirements and acceptance criteria.
2. Identify where the search root is set in the auto extension code.
3. Update logic to use the correct folder as the search root:
   - For rockerc: use the folder opened in the editor.
   - For renv: use ~renv/repo_owner/repo_name/branch/repo_name.
4. Ensure all recursive search and extension detection uses this root.
5. Update logging to show the correct search root.
6. Test with both rockerc and renv scenarios to confirm correct behavior.
7. Commit changes to the extension and spec folder only.
