import os
from opendbc.car.common.basedir import BASEDIR


class TestBasedir:
  def test_basedir_exists(self):
    """Test that BASEDIR points to an existing directory"""
    assert os.path.exists(BASEDIR)
    assert os.path.isdir(BASEDIR)
    
  def test_basedir_is_absolute(self):
    """Test that BASEDIR is an absolute path"""
    assert os.path.isabs(BASEDIR)
    
  def test_basedir_parent_of_common(self):
    """Test that BASEDIR is the parent of the common module"""
    # BASEDIR should be opendbc/car directory
    assert BASEDIR.endswith('car')
    
    # common directory should exist under BASEDIR
    common_dir = os.path.join(BASEDIR, 'common')
    assert os.path.exists(common_dir)
    assert os.path.isdir(common_dir)
    
  def test_basedir_contains_expected_structure(self):
    """Test that BASEDIR contains expected car module structure"""
    # Check for some expected subdirectories
    expected_dirs = ['common', 'tests']
    for dir_name in expected_dirs:
      dir_path = os.path.join(BASEDIR, dir_name)
      assert os.path.exists(dir_path), f"Expected directory {dir_name} not found in BASEDIR"