/*
Camera control thread 
Allows for cam firing parallelism
*/
use rustgeomapping::depth_cam;
use rustgeomapping::data_types::pointcloud::PointCloud;

use std::cell::RefCell;
use std::thread;
use std::sync::mpsc;



///A camera thread which takes a threaded ownership of a camera
/// Thread-safe by design because this is only used when the system control functions relinquish control of the cameras
pub struct CamThread{
    ///The camera Cell (still owned by the system control - its on 'loan')
    cam : RefCell<CamType>,
    ///The id to sort out which camera has sent the pointcloud
    id : u32,
    ///Trigger the camera
    trigger : mpsc::Reciever<bool>,
    ///Get the pointcloud out of the thread
    pcl_out : mpsc::Sender<PointCloud>
}


impl CamThread{

    //Prepares all the variables required for the camera thread
    pub fn prepare(cam : RefCell<CamType>, id : u32, trigger : mpsc::Reciever<bool>, pcl_out : mpsc::Sender<PointCloud>) -> Self{

        Self{
            cam,
            id,
            trigger,
            pcl_out
        }
    }
    
    ///Spin up a thread that the camera uses
    pub fn spin_up(self){

        //Start the thread
        let _ = thread::spawn(||{
            self.cam_loop();
        }
        );

    }

    ///The camera loop 
    fn cam_loop(&self){

        while true{

            if self.trigger.recv() == true{
                //Trigger the measurement
                let pcl = self.cam.borrow_mut().get_pointcloud();

                //Send the measurement
                pcl_out.sened(pcl);

            }else{
                //If the trigger is false, switch off the thread
                break;
            }           

        }
    }


}