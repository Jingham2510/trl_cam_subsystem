//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use rustgeomapping::data_types::pointcloud::PointCloud;
use rustgeomapping::data_types::intrinsic_info::IntrinsicInfo;
use rustgeomapping::computer_vision::get_extrinsic_inv_from_aruco_4x4_250;
use crate::config::config_manager::ConfigManager;
use anyhow::bail;
use nalgebra::{UnitQuaternion, Quaternion, Vector3, Matrix4, Translation3, matrix};
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





//CALIB POS TO WORLD TRANSFORM -----------------

const FRONT_SPOKE_POS : [f32; 3] = [417.67, 2536.85, 1075.80];
const FRONT_SPOKE_ORI : [f32; 4] = [0.00130, -0.11326, 0.99354, 0.00640];
const FRONT_SPOKE_TRANSFORM : Matrix4<f32> = matrix![0.9977538, -0.031512104, -0.059112735,   0.41898492 ;
                                                    0.05408039,    0.8996666,   0.43321505,    -0.258076;
                                                    0.03953024,   -0.4354388,      0.89935,   -1.2794335;
                                                    0.0,          0.0,          0.0,          1.    ];





const BACK_SPOKE_L_POS : [f32; 3] = [203.14, 2065.79, 1075.95];
const BACK_SPOKE_L_ORI : [f32; 4] = [0.00013, 0.06956, 0.99756, 0.00660];
const BACK_SPOKE_L_TRANSFORM : Matrix4<f32> = matrix![0.01105789,   -0.9529749,   0.30284756,  -0.07730941;
                                                      0.9999226,  0.012272284, 0.0021071497,    0.5293364;
                                                      -0.005724692,    0.3028008,    0.9530368,   -1.2832978;
                                                      0.0, 0.0, 0.0, 1.0];



const BACK_SPOKE_R_POS : [f32; 3] = [1033.26, 1724.30, 1316.60];
const BACK_SPOKE_R_ORI : [f32; 4] = [0.02676, 0.17529, -0.98414, 0.00474];
const BACK_SPOKE_R_TRANSFORM : Matrix4<f32> = matrix![0.43745154,   0.7777199, -0.45142877,   1.3026029;
                                                -0.89837676,  0.39998746, -0.18146408,    1.017233 ;
                                                0.03943765,  0.48493487,  0.87366056,   -1.484466;
                                                0.0, 0.0, 0.0, 1.0];

const OG_POS_LIST : [[f32;3] ;3] = [FRONT_SPOKE_POS, BACK_SPOKE_L_POS, BACK_SPOKE_R_POS];
const OG_ORI_LIST : [[f32;4] ;3] = [FRONT_SPOKE_ORI, BACK_SPOKE_L_ORI, BACK_SPOKE_R_ORI];
const CALIB_FRAME_TO_WORLD_TRANSFORM : [Matrix4<f32>; 3] = [FRONT_SPOKE_TRANSFORM , BACK_SPOKE_L_TRANSFORM , BACK_SPOKE_R_TRANSFORM];



///FORCE SENSOR TO CAMERA TRANSFORMS - DEFINED IN THE TCP FRAME - redo translations

const FORCE_TO_FRONT_CAM : Matrix4<f32> = matrix![1.0,  0.0,  0.0, 0.0;
                                                0.0, 1.0,  0.0, 0.30414;
                                                0.0, 0.0, 1.0, 0.06813;
                                                0.0, 0.0, 0.0, 1.0];


const FORCE_TO_BL_CAM: Matrix4<f32> = matrix![1.0000000,  0.0,  0.0,0.26381;
                                            0.0, 1.0,  0.0, -0.15275;
                                            0.0, 0.0, 1.0, 0.06813;
                                            0.0, 0.0, 0.0, 1.0];



const FORCE_TO_BR_CAM : Matrix4<f32> = matrix![1.0,  0.0,  0.0, -0.26377;
                                                0.0, 1.0,  0.0, -0.15275;
                                                0.0, 0.0, 1.0, 0.06813;
                                                0.0, 0.0, 0.0, 1.0];


const LOAD_CELL_TO_CAM :[Matrix4<f32>; 3] = [FORCE_TO_FRONT_CAM, FORCE_TO_BL_CAM, FORCE_TO_BR_CAM];


///TCP POINT TO FORCE SENSOR POINT TRANSFORM - DEFINED IN THE CURRENT TCP FRAME
const SPHERE_TCP_TO_LOAD_CELL : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                            0.0, 1.0, 0.0, 0.0;
                                                            0.0, 0.0, 1.0, 0.35;
                                                            0.0, 0.0, 0.0, 1.0];



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


        self.curr_pos = [233.34, 216.77, 634.11];
        self.curr_ori = [0.00126, -0.11324, 0.99355, 0.00623];

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

            //Calculate the cameras original position
            let calib_pos = OG_POS_LIST[i];
            let calib_ori = OG_ORI_LIST[i];

            // Positions in metres
            let calib_pos_m = Vector3::new(calib_pos[0], calib_pos[1], calib_pos[2]) / 1000.0;
            //Create the original quaternion
            let q_calib = UnitQuaternion::from_quaternion(
                Quaternion::new(calib_ori[0], calib_ori[1], calib_ori[2], calib_ori[3])
            );

            let tcp_at_calib = (Translation3::from(calib_pos_m).to_homogeneous() * q_calib.to_homogeneous());
            let cam_at_calib =   LOAD_CELL_TO_CAM[i] * SPHERE_TCP_TO_LOAD_CELL * tcp_at_calib;

            //Calculate the cameras current position
            let curr_pos_m = Vector3::new(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2]) / 1000.0;
            let q_curr = UnitQuaternion::from_quaternion(
                Quaternion::new(self.curr_ori[0], self.curr_ori[1], self.curr_ori[2], self.curr_ori[3])
            );

            let tcp_at_curr = Translation3::from(curr_pos_m).to_homogeneous() * q_curr.to_homogeneous();
            let cam_at_curr = LOAD_CELL_TO_CAM[i] * SPHERE_TCP_TO_LOAD_CELL * tcp_at_curr;

            //Calculate the transformation from the calibration frame to the current camera frame
            let calib_to_current_transform = cam_at_curr * cam_at_calib.try_inverse().unwrap();


            println!("cam delta to calibration cam pos: {}", calib_to_current_transform.try_inverse().unwrap());
        
            //The point is transformed from the current camera space -> calibration camera space -> world space
            //The camera space is calculated by doing a rigid translationfrom the tcp position/orientation to the position of the camera
            //There is no rotation because this is implicit into the calibration to the world transform

            let sensor_to_world =   CALIB_FRAME_TO_WORLD_TRANSFORM[i] * calib_to_current_transform.try_inverse().unwrap();


            pcl.transform_with(&sensor_to_world);         

            

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

        println!("intrinsics: {:?}", intrinsics);

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
