# Plan: Fix Auto Extension Detection

1. Review current auto-detection logic for extensions.
2. Update detection to:
   - Add `ccache` if any file matches: *.c, *.cc, *.cpp, *.cxx, *.h, *.hpp, *.hxx
   - Add `ros_jazzy` if `package.xml` exists
3. Ensure multiple extensions are detected if multiple file types are present.
4. Run CI (`pixi run ci`) and verify all tests pass.
5. Iterate and fix any remaining issues until CI is green.
6. Commit only the spec and plan folder when done.
