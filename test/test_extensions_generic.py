import unittest
import pytest
import subprocess


@pytest.mark.docker
class TestExtensionsGeneric(unittest.TestCase):
    """Simple tests for deps_rocker extensions via CLI"""

    def setUp(self):
        """Set up test environment"""
        self.base_image = "ubuntu:22.04"
        # Test only working extensions, excluding problematic ones (isaac_sim, ros_humble)
        self.extensions_to_test = [
            "curl",
            "fzf", 
            "git-clone",
            "locales",
            "neovim",
            "tzdata",
            "urdf-viz",
            "uv",
            "vcstool"
        ]

    def test_extensions_individually(self):
        """Test each extension individually via CLI"""
        
        for extension in self.extensions_to_test:
            with self.subTest(extension=extension):
                # Create rocker command for single extension
                cmd = [
                    "pixi", "run", "rocker", 
                    f"--{extension}",
                    "--mode", "dry-run",
                    self.base_image
                ]
                
                try:
                    result = subprocess.run(
                        cmd,
                        capture_output=True,
                        text=True,
                        check=True,
                        timeout=120  # 2 minute timeout per extension
                    )
                    
                    # Verify the extension ran successfully
                    self.assertIsNotNone(result.stdout)
                    self.assertIn("Successfully built", result.stdout)
                    
                except subprocess.CalledProcessError as e:
                    self.fail(f"Extension '{extension}' failed to build. Error: {e.stderr}")
                except subprocess.TimeoutExpired:
                    self.fail(f"Extension '{extension}' timed out during build")

    def test_all_extensions_together(self):
        """Test all extensions together via CLI"""
        
        # Build command with all extensions
        cmd = ["pixi", "run", "rocker"]
        
        for extension in self.extensions_to_test:
            cmd.append(f"--{extension}")
        
        cmd.extend(["--mode", "dry-run", self.base_image])
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
                timeout=600  # 10 minute timeout for all extensions
            )
            
            # Verify all extensions ran successfully together
            self.assertIsNotNone(result.stdout)
            self.assertIn("Successfully built", result.stdout)
            
        except subprocess.CalledProcessError as e:
            self.fail(f"All extensions together failed to build. Error: {e.stderr}")
        except subprocess.TimeoutExpired:
            self.fail("All extensions together timed out during build")


if __name__ == "__main__":
    unittest.main()
