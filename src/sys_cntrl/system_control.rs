//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use rustgeomapping::data_types::pointcloud::PointCloud;
use rustgeomapping::data_types::intrinsic_info::IntrinsicInfo;
use rustgeomapping::computer_vision::get_extrinsic_inv_from_aruco_4x4_250;
use crate::config::config_manager::ConfigManager;
use anyhow::bail;
use nalgebra::{Matrix4, matrix};
use std::io::{Read, stdin};
use std::process::exit;
use std::net::UdpSocket;
use std::{any, fs};


use std::time::{SystemTime, Duration};

use std::ops::{Index, Mul};
use std::{thread};


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





//Transformation from cam to sandbed (with relavent robot position/orientation)

const FRONT_SPOKE_POS : [f32; 3] = [417.67, 2536.85, 1075.80];
const FRONT_SPOKE_ORI : [f32; 4] = [0.00130, -0.11326, 0.99354, 0.00640];
const FRONT_SPOKE_TRANSFORM : Matrix4<f32> = matrix![0.9857188,  -0.0825829, -0.14676017,  0.09638427;
                                                    0.09684497,   0.9909598,  0.09284242,  0.07385595;
                                                    0.13776623,  -0.1057295,  0.98480546, -0.73362076;
                                                    0.0,          0.0,          0.0,          1.    ];



const BACK_SPOKE_L_POS : [f32; 3] = [157.63, 2199.62, 1189.65];
const BACK_SPOKE_L_ORI : [f32; 4] = [0.00056, -0.02994, -0.99950, 0.00966];
const BACK_SPOKE_L_TRANSFORM : Matrix4<f32> = matrix![-0.9975536, 0.009592415, -0.06924424,  0.81535965;
                                                      -0.02083281,  -0.9863254,  0.16348799,   0.4204629;
                                                      -0.06672911,  0.16453058,  0.98411226,  -1.0445058;
                                                      0.0, 0.0, 0.0, 0.99999994];

const BACK_SPOKE_R_POS : [f32; 3] = [740.40, 2015.91, 1212.55];
const BACK_SPOKE_R_ORI : [f32; 4] = [0.00500, 0.45026, -0.89284, 0.00810];
const BACK_SPOKE_R_TRANSFORM : Matrix4<f32> = matrix![0.97652954, 0.025554322,  0.21386217,  0.16984299;
                                                0.07541232,   0.8895182,  -0.4506332,   1.1292337;
                                                -0.20174994,  0.45618448,   0.8667137,  -1.0001591;
                                                0.0, 0.0, 0.0, 1.0];


const OG_POS_LIST : [[f32;3] ;3] = [FRONT_SPOKE_POS, BACK_SPOKE_L_POS, BACK_SPOKE_L_POS];
const OG_ORI_LIST : [[f32;4] ;3] = [FRONT_SPOKE_ORI, BACK_SPOKE_L_ORI, BACK_SPOKE_L_ORI];
const TCP_TRANSFORM_LIST : [Matrix4<f32>; 3] = [FRONT_SPOKE_TRANSFORM, BACK_SPOKE_L_TRANSFORM, BACK_SPOKE_R_TRANSFORM];

//Default croppings for each camera
const FRONT_SPOKE_CROP : [f32;6] = [-999.0, 999.0, -999.0, 999.0, -999.0, 999.0];
const BACK_SPOKE_L_CROP : [f32;6] = [-999.0, 999.0, -999.0, 999.0, -999.0, 999.0];
const BACK_SPOKE_R_CROP : [f32;6] = [-999.0, 999.0, -999.0, 999.0, -999.0, 999.0];

const CROP_LIST : [[f32;6];3] = [FRONT_SPOKE_CROP, BACK_SPOKE_L_CROP, BACK_SPOKE_R_CROP];


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
        }else{
            println!(">{} cameras detected", no_of_cams);
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
        global_hmap.set_all_cells(f32::NAN);


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

    ///Fire all the cameras and perform the workplace transform on each of them
    pub fn fire_and_transform(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{


        self.curr_pos = FRONT_SPOKE_POS;
        self.curr_ori = FRONT_SPOKE_ORI;

         let mut pcl_vec = self.fire_all_cams()?;

        self.workspace_transform(&mut pcl_vec);
     
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

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){  

            let og_pos = OG_POS_LIST[i];
            let og_ori = OG_ORI_LIST[i];

            //Get the delta position and orientation
            let delta_pos : [f32;3] = [(og_pos[0] - self.curr_pos[0]) /1000.0, (og_pos[1] - self.curr_pos[1]) /1000.0, (og_pos[2] - self.curr_pos[2]) / 1000.0];
            
            //Get the quaternion that rotates to the original calibration orientation
            let delta_ori : [f32;4] = {
                //Invert the current orientation
                let inv_curr_q = [self.curr_ori[0], -self.curr_ori[1], -self.curr_ori[2], -self.curr_ori[3]];

                //The delta is equal to the end orientation multiplied by the inverse of the start orientation
                [  og_ori[0]*inv_curr_q[0] - og_ori[1]*inv_curr_q[1] - og_ori[2]*inv_curr_q[2] - og_ori[3]*inv_curr_q[3],
                   og_ori[0]*inv_curr_q[1] + og_ori[1]*inv_curr_q[0] - og_ori[2]*inv_curr_q[3] + og_ori[3]*inv_curr_q[2], 
                   og_ori[0]*inv_curr_q[2] + og_ori[1]*inv_curr_q[3] + og_ori[2]*inv_curr_q[0] - og_ori[3]*inv_curr_q[1],
                   og_ori[0]*inv_curr_q[3] - og_ori[1]*inv_curr_q[2] + og_ori[2]*inv_curr_q[1] + og_ori[3]*inv_curr_q[0]]

            };
            
            
            //Split quaternion for readability
            let [q_w, q_i, q_j, q_k] = delta_ori;   

            //Square the values required
            let q_i_sq = q_i.powi(2);
            let q_j_sq = q_j.powi(2);
            let q_k_sq = q_k.powi(2);
        

            //Calculate the workspace transform
            let work_tmat = matrix![1.0 - 2.0*(q_j_sq + q_k_sq), 2.0*(q_i*q_j - q_k*q_w), 2.0*(q_i*q_k + q_j*q_w), delta_pos[0];
                                                                        2.0*(q_i*q_j + q_k*q_w), 1.0 - 2.0*(q_i_sq + q_k_sq), 2.0*(q_j*q_k - q_i*q_w), delta_pos[1];
                                                                        2.0*(q_i*q_k - q_j*q_w), 2.0*(q_j*q_k + q_i*q_w), 1.0 - 2.0*(q_i_sq + q_j_sq), delta_pos[2];
                                                                        0.0, 0.0, 0.0, 1.0];

            

                                                                
                                                            
                
            //Combine the standard transform and the position based transform            
            let tmat = TCP_TRANSFORM_LIST[i].mul(work_tmat);

            println!("{}", tmat);


            pcl.transform_with(&tmat);

        }
    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - WARNING - DO NOT TYPE");


        //Create a new network listener
        let mut stream = UdpSocket::bind("0.0.0.0:8080")?;
        stream.connect("192.168.55.100:8080")?;

        let mut buf : [u8; 10] = [0;10]; 

        loop{
            let n = stream.recv(buf.as_mut_slice())?;

            let inp = str::from_utf8(&buf[..n])?;

            if inp == "CONNECT?"{
                stream.send(b"YES");
                break;
             }
        }
        
        //Do until main system instructs to stop
        loop{           
            
                let mut buf : [u8;1024] = [0; 1024];
                let n = stream.recv(buf.as_mut_slice())?;
                let inp = str::from_utf8(&buf[..n])?;


                match inp{
                    //Close the connection
                    "QUIT!" | "CLOSE!" => {
                        println!(">Closing auto system");
                        break;
                    }

                    "GLOBAL_SIZE?" =>{                        
                        let size = format!("{},{}", self.global_hmap.width(), self.global_hmap.height());

                        stream.send(&size.into_bytes())?;
                    }

                    "CLOSE?" =>{
                        println!("GRACEFULLY EXITING");
                        exit(1)
                    }

                    //Assume other messages are position/orientation instructions
                    _ => {if !self.parse_pos_ori(inp).is_ok(){
                            //println!(">{}", inp);
                            //println!(">Invalid pos/ori string")
                        }else{
                            //Only fire all cameras if the main system has sent a pos string - stops the and doesnt risk file being read while incomplete                      
                       

                            //Fire all cameras
                            let mut pcl_list = self.fire_all_cams()?;

                            //Crop the point cloud
                            self.standard_crop(&mut pcl_list);

                            //Go through each point cloud and transform it to the work space
                            self.workspace_transform(&mut pcl_list);    


                            //Group the pointclouds and turn them into a heightmap - resolution based on desired resolution
                            let local_hmap = Heightmap::create_from_pcl_list_with_res(pcl_list, HMAP_RES)?;
                            //local_hmap.save_to_file("/home/trl/Desktop/local");
                            
                            //Slot the heightmap into the global heightmap
                            self.global_hmap.update_section(local_hmap)?;


                            //Update the current heightmap file
                            //self.global_hmap.save_to_file(HMAP_FP)?;

                            let flattened_cells = self.global_hmap.get_flattened_cells()?;
                            
                            //Turn the list of floats into a list of bytes
                            let bytes : Vec<u8> = flattened_cells.into_iter().flat_map(|i| i.to_be_bytes()).collect();

                            //Tell the main system how many pckets there will be 
                            const PACKET_SIZE : usize = 512;
                            let no_of_packets =bytes.len()/PACKET_SIZE;

                            stream.send(&format!("{}", no_of_packets).into_bytes())?;

                            for i in 0..no_of_packets{
                                if i == no_of_packets{
                                    stream.send(&bytes[(i*PACKET_SIZE)..])?;
                                }else{                                                         
                                    stream.send(&bytes[(i*PACKET_SIZE)..((i*PACKET_SIZE + PACKET_SIZE))])?;

                                    let mut buf : [u8; 10] = [0;10]; 
                                    //Wait for packet confirmation
                                    loop{
                                        let n = stream.recv(buf.as_mut_slice())?;
                                        let inp = str::from_utf8(&buf[..n])?;
                                        if inp == "NEXT"{
                                            break;
                                        }
                                    }

                                }
                            }
                            

                        };  
                    }     
            
                    
                }
            }

            Ok(())
        }



    
    ///Calculate inverse extrinsic matrices based on aruco tag captures
    /// Assumes that the cameras are already in the correct position
    /// Also predefined for the board used in the TRL lab
    pub fn calc_calib_mats(&mut self, delete_calib_imgs : bool) -> Result<(), anyhow::Error>{
        
        //For each camera get the intrinsic matrix
        let intrinsics = self.get_all_intrinsics()?;
        //Setup the filepath
        let fp = "temp_aruco_calibration";
        //For each camera take a colour image
        let img_filepaths = self.fire_all_cams_image(fp)?; 


        //ARUCO BOARD SETUP-----------------------------
        //Center to center distance
        const BOARD_SIZE : f32 = 0.797;
        //Board to sand distance
        const BOARD_THICKNESS : f32 = 0.0185;
        const MARKER_COORDS : [[f32; 3]; 4] = [[0.0, 0.0, BOARD_THICKNESS], [BOARD_SIZE, 0.0, BOARD_THICKNESS],  [BOARD_SIZE, BOARD_SIZE, BOARD_THICKNESS], [0.0, BOARD_SIZE, BOARD_THICKNESS]];
        const MARKER_IDS : [i32;4] = [1, 3, 2, 0];
        //For each image calculate the inverse extrinsics 
        for (i, image) in img_filepaths.iter().enumerate(){
            println!(">----------CAM: {}-------------", i);

            if let Ok(extrinsic_inv) = get_extrinsic_inv_from_aruco_4x4_250(&image, MARKER_IDS.to_vec(), MARKER_COORDS.to_vec(), &intrinsics[i]){
                println!(">-----extrinsics-----");
                println!(">{}", extrinsic_inv.try_inverse().unwrap());
                
                println!(">-----inverse extrinsics-----");
                println!(">{}", extrinsic_inv);
            }else{
                println!(">Failed to calc extrinsics for cam");
            };


              //If delete is true - delete the images
            if delete_calib_imgs{
                fs::remove_file(image)?;
            }

        }
        
      


        Ok(())
    }

    ///Get each connected cameras intrinsic matrix
    fn get_all_intrinsics(&self) -> Result<Vec<IntrinsicInfo>, anyhow::Error>{

        //Request the intrinsic matrix info from the camera
        let mut intrinsics : Vec<IntrinsicInfo> = vec![];

        for cam in self.cameras.iter(){
            intrinsics.push(cam.get_intrinsics()?)
        }

        Ok(intrinsics)
    }

    ///Get every camera to take an rgbd image
    pub fn fire_all_cams_image(&mut self, base_filepath : &str) -> Result<Vec<String>, anyhow::Error>{
        
        let mut filepaths : Vec<String> = vec![];
        
        //Create each image and label it according to its number in the id
        for cam in self.cameras.iter_mut(){
            let img_fp = format!("{}_{}", base_filepath, cam.id());            
            filepaths.push(cam.get_colour_image(&img_fp)?);

            println!("Cam {} fired and saved", cam.id());

        }
        Ok(filepaths)
    }

    
    ///Parse a position/ori message through std in
    /// To minimise computation it is formated minimally and minimal error checking is done
    /// x,y,z,qw,qx,qy,qz
    /// 1.0,2.0,3.0,4.0,5.0,6.0,7.0
    fn parse_pos_ori(&mut self, pos_ori_str : &str) -> Result<(), anyhow::Error>{


        let tokens : Vec<&str> = pos_ori_str.split(",").collect();

        if tokens.len() != 7{
            println!("{:?}", tokens);
            bail!("Invalid pos/ori string")
        }

        self.curr_pos = [tokens[0].parse()?, tokens[1].parse()?, tokens[2].parse()?];
        self.curr_ori = [tokens[3].parse()?, tokens[4].parse()?, tokens[5].parse()?, tokens[6].parse()?];    


        Ok(())
    }  


    ///Reset all connected hardware
    pub fn reset_hardware() -> Result<(), anyhow::Error>{
        RealsenseCam::reset_all()
    }



}
