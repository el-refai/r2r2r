import os
import h5py
import numpy as np
from typing import Optional
import tyro

class NumpyToH5:
    def __init__(self, save_dir: str, filename: str):
        """Initialize NumpyToH5 converter.
        
        Args:
            save_dir: Directory to save the H5 file
            filename: Name of the H5 file
        """
        self.save_dir = save_dir
        self.filename = filename
        self.h5_file = os.path.join(save_dir, filename)
        
    def save_array(self, array: np.ndarray, dataset_name: str, chunks: bool = True):
        """Save numpy array to H5 file.
        
        Args:
            array: Numpy array to save
            dataset_name: Name of the dataset in H5 file
            chunks: Whether to enable chunking for the dataset
        """
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        array = np.asarray(array)
        if array.ndim == 1:
            array = array[np.newaxis, :]
            
        with h5py.File(self.h5_file, 'a') as f:
            if dataset_name not in f:
                f.create_dataset(dataset_name, 
                               data=array,
                               maxshape=(None, array.shape[1]),
                               chunks=chunks)
            else:
                dset = f[dataset_name]
                current_size = dset.shape[0]
                dset.resize(current_size + array.shape[0], axis=0)
                dset[current_size:] = array
                
    def read_array(self, dataset_name: str) -> Optional[np.ndarray]:
        """Read numpy array from H5 file.
        
        Args:
            dataset_name: Name of the dataset to read
            
        Returns:
            Numpy array if dataset exists, None otherwise
        """
        if not os.path.exists(self.h5_file):
            return None
            
        with h5py.File(self.h5_file, 'r') as f:
            if dataset_name not in f:
                return None
            return f[dataset_name][:]

# TODO: cleanup path!
def main(path: str = '/home/lifelong/xi/data/camera_poses/camera_poses_calib_extr.h5', dataset: str = 'poses'):
    
    save_dir = os.path.dirname(path)
    filename = os.path.basename(path)
    
    h5_reader = NumpyToH5(save_dir, filename)
    array = h5_reader.read_array(dataset)
    
    if array is not None:
        print(f"Shape of array: {array.shape}")
        print(f"Content of array:\n{array}")
    else:
        print(f"Dataset {dataset} not found in file {path}")

def main_write(path: str = '/home/yujustin/xi/data/camera_poses/camera_poses_wrists_ego2.h5', 
                   array: np.ndarray = np.array([
                       [0.73151284, -0.01318118,  0.68094802, -0.16910131, -0.69683933,  0.6796996, 0.15436359],
                       [0.96877319,  0.41455391,  0.53939092, -0.19296113,  0.33873037,  0.80304313, -0.45072111],
                       [0.73151284, -0.01318118,  0.68094802, -0.16910131, -0.69683933,  0.6796996, 0.15436359]
                       ]
                   )):
        save_dir = os.path.dirname(path)
        filename = os.path.basename(path)
        
        h5_writer = NumpyToH5(save_dir, filename)
        h5_writer.save_array(array, 'poses')
        print(f"Saved array of shape {array.shape} to {path}")

def main_read(path: str = '/home/yujustin/xi/data/camera_poses/camera_poses_calib_extr.h5'):
    
    with h5py.File(path, 'r') as f:
        if 'poses' in f:
            poses = f['poses'][:]
            print(f"Shape of poses array: {poses.shape}")
            print("All poses:")
            print(poses)
            
            # Check for duplicates
            unique_poses = np.unique(poses, axis=0)
            print(f"\nNumber of unique poses: {len(unique_poses)}")
            print("Unique poses:")
            print(unique_poses)
            
if __name__ == '__main__':
    tyro.cli(main_read())

