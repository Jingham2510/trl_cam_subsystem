//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use rustgeomapping::data_types::pointcloud::PointCloud;
use crate::config::config_manager::ConfigManager;
use anyhow::bail;
use nalgebra::{Matrix4, matrix};
use std::io::stdin;

use std::time::{SystemTime};

use std::ops::Mul;
use std::{thread};

use tokio::sync::watch;

pub struct SystemController{
    ///The camera objects that are plugged in 
    cameras : Vec<CamType>,
    ///The current heightmap of the system
    global_hmap : Heightmap,

    //Current position (tcp mm from the base of the robot)
    curr_pos : [f32; 3],
    //Current orientation (tcp quartenion from the base of the robot)
    curr_ori : [f32; 4]
}


//Filepath for height map saving
const HMAP_FP : &str = "/home/trl/Desktop/global";


//Heightmap size controllers - hmap size based on a resolution of 0.0015m over a space of 1.5m?
const HMAP_RES : f32 = 0.0015;
const GLOBAL_AREA_WIDTH : f32 = 1.5;
const GLOBAL_AREA_HEIGHT : f32 = 1.5;


const GLOBAL_HMAP_WIDTH : usize = (GLOBAL_AREA_WIDTH / HMAP_RES) as usize;
const GLOBAL_HMAP_HEIGHT : usize = (GLOBAL_AREA_HEIGHT / HMAP_RES) as usize;





//The transformation from the cameras to the tcp - need to be sorted
const CAM_A_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 0.0;
                                                0.0, 0.0, 0.0, 1.0];

const CAM_BL_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 0.0;
                                                0.0, 0.0, 0.0, 1.0];

const CAM_BR_TRANSFORM : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                0.0, 1.0, 0.0, 0.0;
                                                0.0, 0.0, 1.0, 0.0;
                                                0.0, 0.0, 0.0, 1.0];

const TCP_TRANSFORM_LIST : [Matrix4<f32>; 3] = [CAM_A_TRANSFORM, CAM_BL_TRANSFORM, CAM_BR_TRANSFORM];

//Default croppings for each camera
const CAM_A_CROP : [f32;6] = [-0.15, 0.4, 0.05, 0.13, 0.3, 1.0];
const CAM_BL_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, 0.0, 1.0];
const CAM_BR_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, 0.0, 1.0];

const CROP_LIST : [[f32;6];3] = [CAM_A_CROP, CAM_BL_CROP, CAM_BR_CROP];


impl SystemController{

    ///Starts the system, taking control away from the user
    pub fn start_system_control(config : &mut ConfigManager) -> Result<Self, anyhow::Error>{

        

        println!(">Starting system control - no longer accepting typed user input");
        println!(">GLOBAL WIDTH:{} GLOBAL HEIGHT:{}", GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_WIDTH);

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
            if cam == "Realsense"{
                connected_cams.push(CamType::RealsenseCam(DepthCam::connect_realsense(realsense_cnt)?));
                realsense_cnt += 1;
            }
        }

        let mut global_hmap = Heightmap::new(GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_HEIGHT);
        global_hmap.set_lower_coord_bounds([0.0, 0.0]);
        global_hmap.set_upper_coord_bounds([1.5, 1.5]);

        Ok(SystemController{
            cameras : connected_cams,
            global_hmap,
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

    ///Performs the combined default-workplace transform on a set of pointclouds
    fn workspace_transform(&self, pcl_list : &mut Vec<PointCloud>){


        let [q_0, q_1, q_2, q_3] = self.curr_ori;   

        let q_0_sq = q_0.powi(2);
        let q_1_sq = q_1.powi(2);
        let q_2_sq = q_2.powi(2);
        let q_3_sq = q_3.powi(2);


        //Calculate the workspace transform
        let work_tmat = matrix![2.0*(q_0_sq + q_1_sq) - 1.0, 2.0*(q_1*q_2-q_0*q_3), 2.0*(q_1*q_3+q_0*q_2), self.curr_pos[0];
                                                                    2.0*(q_1*q_2 + q_0*q_3), 2.0*(q_0_sq*q_2_sq) - 1.0, 2.0*(q_1*q_3 - q_0*q_1), self.curr_pos[1];
                                                                    2.0*(q_1*q_3 - q_0*q_2), 2.0*(q_2*q_3 + q_0*q_1), 2.0*(q_0_sq + q_3_sq) - 1.0, self.curr_pos[2];
                                                                    0.0, 0.0, 0.0, 1.0];

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){          
            //Combine the standard transform and the position based transform            
            let tmat = TCP_TRANSFORM_LIST[i].mul(work_tmat);
            
            pcl.transform_with(&tmat);
        }
    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - WARNING - DO NOT TYPE");

        //Alert the main system to the filepath of the heightmap file
        println!(">FP:{}", HMAP_FP);

        //Spawn a std in reciever
        let mut stdin_channel = Self::spawn_auto_stdin_channel();

        

        //Do until main system instructs to stop
        loop{           
            
            //Poll stdin to see if the main system has updated current position/orientation
            if stdin_channel.has_changed()?{
                let inp = stdin_channel.borrow_and_update().to_lowercase();
                let inp = inp.trim();
                match inp{
                    //Close the connection
                    "quit" | "close" => {
                        println!(">Closing auto system");
                        break;
                    }
                    //Assume other messages are position/orientation instructions
                    _ => {if !self.parse_pos_ori(inp).is_ok(){
                        println!(">Invalid pos/ori string")
                        }else{
                            //Only fire all cameras if the main system has sent a pos string - stops the and doesnt risk file being read while incomplete

                            println!(">FIRING");
                            
    
                            //Fire all cameras
                            let mut pcl_list = self.fire_all_cams()?;

                            //Crop the point cloud
                            self.standard_crop(&mut pcl_list);

                           // let temp_hmap = Heightmap::create_from_pcl_list_with_res(pcl_list, HMAP_RES)?;
                           // temp_hmap.save_to_file("/home/trl/Desktop/untransformed_local");

                            let now = SystemTime::now();
                            //Go through each point cloud and transform it to the work space
                            self.workspace_transform(&mut pcl_list);    
                            pcl_list[0].save_to_file("/home/trl/Desktop/local_pcl");
                            println!("bnds: {:?}", pcl_list[0].bounds());
                            println!(">TRANSFORM_TIME: {:?}", now.elapsed()?.as_millis());        

                            //Group the pointclouds and turn them into a heightmap - resolution based on desired resolution
                            let local_hmap = Heightmap::create_from_pcl_list_with_res(pcl_list, HMAP_RES)?;
                            local_hmap.save_to_file("/home/trl/Desktop/local");

                            println!(">HEIGHTMAP_CREATION_TIME: {:?}", now.elapsed()?.as_millis());
                            println!("ROWS:{}, COL:{}", local_hmap.height(), local_hmap.width());

                            
                            //Slot the heightmap into the global heightmap
                            println!("{:?}", self.global_hmap.update_section(local_hmap));
                            println!(">SLOTTING_TIME: {:?}", now.elapsed()?.as_millis());       

                            //Update the current heightmap file
                            self.global_hmap.save_to_file(HMAP_FP)?;
                            println!(">SAVING_TIME: {:?}", now.elapsed()?.as_millis());

                            println!(">READ");

                        };  
                    }                    
                    
                }
            }            

        }


        Ok(())
    }


    ///Spawns a blocking stdin thread (blocks a different thread)
    fn spawn_auto_stdin_channel() -> watch::Receiver<String> {
        let (tx, rx) = watch::channel(String::new());
        thread::spawn(move || loop {
            let mut buffer = String::new();
            stdin().read_line(&mut buffer).unwrap();
            tx.send(buffer).unwrap();
        });
    rx
    }

    ///Parse a position/ori message through std in
    /// To minimis computation it is formated minimally and minimal error checking is done
    /// x,y,z,qw,qx,qy,qz
    /// 1.0,2.0,3.0,4.0,5.0,6.0,7.0
    fn parse_pos_ori(&mut self, pos_ori_str : &str) -> Result<(), anyhow::Error>{


        let tokens : Vec<&str> = pos_ori_str.split(",").collect();

        if tokens.len() != 7{
            bail!("Invalid pos/ori string")
        }

        self.curr_pos = [tokens[0].parse()?, tokens[1].parse()?, tokens[2].parse()?];
        self.curr_ori = [tokens[3].parse()?, tokens[4].parse()?, tokens[5].parse()?, tokens[6].parse()?];

    


        Ok(())
    }

  

}
