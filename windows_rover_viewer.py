import pybullet as p
import pybullet_data
import time
import os
import xml.etree.ElementTree as ET

def load_windows_rover():
    print("Starting Windows PyBullet Rover Viewer...")
    
    # Start PyBullet GUI
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Setup camera and environment
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0])
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "src", "rover_description", "urdf", "rover.urdf")
    temp_urdf_path = os.path.join(script_dir, "temp_windows_rover.urdf")
    
    # Fix paths for Windows PyBullet (replace package:// with absolute paths)
    print("Processing URDF meshes for Windows...")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    mesh_pkg_prefix = "package://rover_description/"
    abs_mesh_dir = os.path.join(script_dir, "src", "rover_description").replace("\\", "/") + "/"
    
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if filename and filename.startswith(mesh_pkg_prefix):
            new_filename = filename.replace(mesh_pkg_prefix, abs_mesh_dir)
            mesh.set('filename', new_filename)
            
    tree.write(temp_urdf_path)
    
    # Load the rover
    print("Loading Rover into Simulation...")
    try:
        rover_id = p.loadURDF(temp_urdf_path, [0, 0, 0.5], useFixedBase=False)
        print("✅ Rover Loaded Successfully!")
        
        # Keep simulation running
        print("Simulation running. Close the window or press Ctrl+C to exit.")
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1./240.)
            
    except Exception as e:
        print(f"❌ Failed to load URDF: {e}")
        print("Note: PyBullet on Windows sometimes struggles with .dae (Collada) files if it wasn't built with Assimp support.")
        print("If you see errors about missing meshes, try converting your .dae files to .obj or .stl.")
    finally:
        if os.path.exists(temp_urdf_path):
            os.remove(temp_urdf_path)

if __name__ == '__main__':
    load_windows_rover()
