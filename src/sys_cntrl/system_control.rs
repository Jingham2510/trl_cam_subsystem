//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use crate::config::config_manager::ConfigManager;
use anyhow::bail;

pub struct SystemController{
    ///The camera objects that are plugged in 
    cameras : Vec<CamType>,
    ///The current heightmap of the system
    global_hmap : Heightmap
}


//Heightmap size controllers
const GLOBAL_HMAP_WIDTH : usize = 75;
const GLOBAL_HMAP_HEIGHT : usize = 75;

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
            global_hmap : Heightmap::new(GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_HEIGHT)
        })
    }


    ///Fire all of the depth cameras the system controls and saves the pointclouds
    pub fn fire_all_cams(&mut self) -> Result<(), anyhow::Error>{

        let mut cnt = 0;

        for cam in self.cameras.iter_mut(){
            
            let pcl = cam.take_pcl()?;

            println!("{:?}", pcl.save_to_file(&format!("test_{}", cnt))?);

            cnt +=1 
        }
        Ok(())
    }

}
