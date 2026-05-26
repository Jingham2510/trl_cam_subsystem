//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use rustgeomapping::data_types::pointcloud::PointCloud;
use crate::config::config_manager::ConfigManager;
use anyhow::bail;
use nalgebra::{Matrix4, matrix};

pub struct SystemController{
    ///The camera objects that are plugged in 
    cameras : Vec<CamType>,
    ///The current heightmap of the system
    global_hmap : Heightmap,

    //Current position (tcp mm from the base of the robot)
    curr_pos : [f64; 3],
    //Current orientation (tcp quartenion from the base of the robot)
    curr_ori : [f64; 4]
}


//Heightmap size controllers
const GLOBAL_HMAP_WIDTH : usize = 1000;
const GLOBAL_HMAP_HEIGHT : usize = 1000;


//The local heightmap - although it maybe should be calculated based on the bounds?
const LOCAL_HMAP_WIDTH : usize = 75;
const LOCAL_HMAP_HEIGHT : usize = 75;


//The transformation from the cameras to the tcp - need to be sorted
const CAM_A_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 1.0;
                                                0.0, 0.0, 0.0, 1.0];

const CAM_BL_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 1.0;
                                                0.0, 0.0, 0.0, 1.0];

const CAM_BR_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 1.0;
                                                0.0, 0.0, 0.0, 1.0];

const TRANSFORM_LIST : [Matrix4<f32>; 3] = [CAM_A_TRANSFORM, CAM_BL_TRANSFORM, CAM_BR_TRANSFORM];

//Default croppings for each camera
const CAM_A_CROP : [f32;6] = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0];
const CAM_BL_CROP : [f32;6] = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0];
const CAM_BR_CROP : [f32;6] = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0];

const CROP_LIST : [[f32;6];3] = [CAM_A_CROP, CAM_BL_CROP, CAM_BR_CROP];


impl SystemController{

    ///Starts the system, taking control away from the user
    pub fn start_system_control(config : &mut ConfigManager) -> Result<Self, anyhow::Error>{

        

        println!(">Starting system control - no longer accepting typed user input");

        //Update the config - not required by the system as it won't be updated while the system is alive
        config.update();

        //Check to make sure there are cameras to connect to 
        let no_of_cams = config.no_of_cams();
        if no_of_cams == 0{
            bail!(">No cameras to control")
        }

        //If valid cameras - 
        let cam_list = config.cams();
        let mut connected_cams : Vec<CamType> = vec![];

        let mut realsense_cnt = 0;

        for cam in cam_list{
            println!("{cam}");
            if cam == "Realsense"{
                connected_cams.push(CamType::RealsenseCam(DepthCam::connect_realsense(realsense_cnt)?));
                realsense_cnt += 1;
            }
        }

        Ok(SystemController{
            cameras : connected_cams,
            global_hmap : Heightmap::new(GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_HEIGHT),
            curr_pos : [0.0, 0.0, 0.0],
            curr_ori : [0.0, 0.0, 0.0, 0.0]
        })
    }


    ///Fire all of the depth cameras the system controls and saves the pointclouds
    pub fn fire_all_cams(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{

    
        let mut pcl_vec : Vec<PointCloud> = vec![];

        for (i, cam) in self.cameras.iter_mut().enumerate(){     
            
            pcl_vec.push(cam.take_pcl()?);
        }
        Ok(pcl_vec)
    }

    ///Performs the default crop on a list of pointclouds
    fn standard_crop(&self, pcl_list : &mut Vec<PointCloud>){

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){
            let crop = CROP_LIST[i];
            pcl.crop(crop[0], crop[1], crop[2], crop[3], crop[4], crop[5]);
        }

    }

    ///Performs the default crop on a list of pointclouds
    fn standard_transform(&self, pcl_list : &mut Vec<PointCloud>){

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){
            
            
            pcl.transform_with(&TRANSFORM_LIST[i]);
        }

    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - Warning - do not type");

        //Alert the main system to the filepath of the heightmap file

        //Do until main system instructs to stop
        loop{
            
            //Poll stdin to see if the main system has updated current position/orientation

            //Fire all cameras
            let mut pcl_list = self.fire_all_cams()?;

            //Crop the point cloud
            self.standard_crop(&mut pcl_list);

            //Go through each point cloud and transform it to the correct space
            self.standard_transform(&mut pcl_list);            

            //Group the pointclouds and turn them into a heightmap
            let local_hmap = Heightmap::create_from_pcl_list(pcl_list, LOCAL_HMAP_WIDTH, LOCAL_HMAP_HEIGHT)?;

            //Slot the heightmap into the global heightmap

            //Update the current heightmap file


        }


        Ok(())
    }

}
