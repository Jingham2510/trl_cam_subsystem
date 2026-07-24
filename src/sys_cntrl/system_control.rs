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

const FRONT_SPOKE_POS : [f32; 3] = [396.02, 2539.15, 1347.70];
const FRONT_SPOKE_ORI : [f32; 4] = [0.00128, -0.11323, 0.99355, 0.00628];
const FRONT_SPOKE_TRANSFORM : Matrix4<f32> = matrix![0.99772996, -0.030186119, -0.060197674,   0.39343914;
                                                    0.053487673,    0.8983324,    0.4360481,  -0.26534083;
                                                    0.040914923,   -0.4382781,   0.89790773,   -1.3016539;
                                                    0.0,          0.0,          0.0,          1.    ];





const BACK_SPOKE_L_POS : [f32; 3] = [250.29, 2206.10, 1470.51];
const BACK_SPOKE_L_ORI : [f32; 4] = [0.00251, 0.08842, 0.99606, 0.00595];
const BACK_SPOKE_L_TRANSFORM : Matrix4<f32> = matrix![-0.010600714,  -0.96143806,    0.2748172,  -0.03968785;
                                                      0.9995037, -0.018341642, -0.025613029,   0.37003002;
                                                      0.029665936,   0.27440926,    0.9611553,   -1.4316759;
                                                      0.0, 0.0, 0.0, 1.0];



const BACK_SPOKE_R_POS : [f32; 3] = [817.92, 2205.33, 1470.68];
const BACK_SPOKE_R_ORI : [f32; 4] = [0.00062, 0.39734, -0.91765, -0.00649];
const BACK_SPOKE_R_TRANSFORM : Matrix4<f32> = matrix![-0.0077567315,     0.8894256,   -0.45701447,     1.1159229;
                                                -0.9996761,  -0.017976763,   -0.01801863,    0.41367507;
                                                -0.024241868,    0.45672664,    0.88927686,     -1.400311;
                                                0.0, 0.0, 0.0, 1.0];

const OG_POS_LIST : [[f32;3] ;3] = [FRONT_SPOKE_POS, BACK_SPOKE_L_POS, BACK_SPOKE_R_POS];
const OG_ORI_LIST : [[f32;4] ;3] = [FRONT_SPOKE_ORI, BACK_SPOKE_L_ORI, BACK_SPOKE_R_ORI];
const T_WORLD_CALIB : [Matrix4<f32>; 3] = [FRONT_SPOKE_TRANSFORM , BACK_SPOKE_L_TRANSFORM , BACK_SPOKE_R_TRANSFORM];



///FORCE SENSOR TO CAMERA TRANSFORMS - DEFINED IN THE TCP FRAME - CAD FORMULATED
/*
const FORCE_TO_FRONT_CAM : Matrix4<f32> = matrix![1.0,  0.0,  0.0, 0.0;
                                                0.0000000, -0.9396926, -0.3420202, 0.30414;
                                                0.0000000,  0.3420202, -0.9396926, 0.06813;
                                                0.0, 0.0, 0.0, 1.0];


const FORCE_TO_BL_CAM: Matrix4<f32> = matrix![0.5000000,  0.8660254,  0.0000000, 0.26381;
                                            0.8137977, -0.4698463,  0.3420202, -0.15275;
                                            0.2961981, -0.1710101, -0.9396926, 0.06813;
                                            0.0, 0.0, 0.0, 1.0];



const FORCE_TO_BR_CAM : Matrix4<f32> = matrix![0.5000000, -0.8660254,  0.0000000, -0.26377;
                                                -0.8137977, -0.4698463,  0.3420202, -0.15275;
                                                -0.2961981, -0.1710101, -0.9396926, 0.06813;
                                                0.0, 0.0, 0.0, 1.0];

*/

///FORCE SENSOR TO CAMERA TRANSFORMS - DEFINED IN THE TCP FRAME -opencv calced

const FORCE_TO_FRONT_CAM : Matrix4<f32> = matrix![-0.9666492150824095, 0.23188556567066318, 0.108712370106434, -0.05209161914267685;
-0.2560411242103606, -0.8656119923052741, -0.4303008499763727, 0.3042019624662642;
-0.005677824729279683, -0.44378481633657846, 0.8961153938502859, -0.07071758541125878;
0.0, 0.0, 0.0, 1.0;];


const FORCE_TO_BL_CAM: Matrix4<f32> = matrix![-0.34793504505655815, 0.8802569673851697, -0.3226280765701508, 0.2727153062233094;
-0.9375145913928484, -0.3256708727680504, 0.12249438173237481, -0.08424829750452015;
0.0027559657093831624, 0.3450886176047546, 0.9385660608889759, -0.07207150207100577;
0.0, 0.0, 0.0, 1.0;];



const FORCE_TO_BR_CAM : Matrix4<f32> = matrix![-0.692879913820384, -0.6579935601432695, 0.29489303117273324, -0.202532282772387;
0.7210519218831212, -0.6329897706451473, 0.2817943864012575, -0.23164233477668095;
0.0012453806303291681, 0.40788285704180977, 0.9130334188618159, -0.0722924366162144;
0.0, 0.0, 0.0, 1.0;];



const T_LC_CAM :[Matrix4<f32>; 3] = [FORCE_TO_FRONT_CAM, FORCE_TO_BL_CAM, FORCE_TO_BR_CAM];


///TCP POINT TO FORCE SENSOR POINT TRANSFORM - DEFINED IN THE CURRENT TCP FRAME
/// USE FC TO TCP DO NOT MATCH THE TCP SPECIFIED IN RAPID
const T_STCP_LC : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                            0.0, 1.0, 0.0, 0.0;
                                                            0.0, 0.0, 1.0, 0.45;
                                                            0.0, 0.0, 0.0, 1.0];

const T_NOTOOL_LC: Matrix4<f32> =  matrix![1.0, 0.0, 0.0, 0.0;
                                                            0.0, 1.0, 0.0, 0.0;
                                                            0.0, 0.0, 1.0, 0.0;
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


        self.curr_pos = [75.65, 1805.87, 516.47];
        self.curr_ori = [0.00126, -0.11322, 0.99355, 0.00617];


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



            //THIS IS THE PROBLEM----------------------- as the calibration pose is correct

            // Positions in metres
            let calib_pos_m = Vector3::new(calib_pos[0], calib_pos[1], calib_pos[2]) / 1000.0;
            //Create the original quaternion
            let q_calib = UnitQuaternion::from_quaternion(
                Quaternion::new(calib_ori[0], calib_ori[1], calib_ori[2], calib_ori[3])
            );

            let tcp_at_calib = Translation3::from(calib_pos_m).to_homogeneous() * q_calib.to_homogeneous();
        
            let cam_at_calib =   tcp_at_calib * T_NOTOOL_LC  *  T_LC_CAM[i];

            //Calculate the cameras current position
            let curr_pos_m = Vector3::new(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2]) / 1000.0;
            let q_curr = UnitQuaternion::from_quaternion(
                Quaternion::new(self.curr_ori[0], self.curr_ori[1], self.curr_ori[2], self.curr_ori[3])
            );


            let tcp_at_curr = Translation3::from(curr_pos_m).to_homogeneous() * q_curr.to_homogeneous();
            let cam_at_curr = tcp_at_curr * T_NOTOOL_LC *  T_LC_CAM[i];

            //Calculate the transformation from the calibration frame to the current camera frame
            let T_calib_curr= cam_at_calib.try_inverse().unwrap() * cam_at_curr;
            


            println!("cam delta to calibration cam pos: {}", T_calib_curr);

            //--------------------
        
            //The point is transformed from the current camera space -> calibration camera space -> world space
            //The camera space is calculated by doing a rigid transformation from the tcp position/orientation to the position of the camera
        

            let T_world_curr =   T_WORLD_CALIB[i] * T_calib_curr;

            println!("Final transform: {}", T_world_curr);


            pcl.transform_with(&T_world_curr);         

            

        }
    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - WARNING - DO NOT TYPE");

        const PCL_DEBUG :bool = true;
        let mut pcl_cnt = 0;


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

                            if PCL_DEBUG{
                                for pcl in &pcl_list{
                                    let fp = format!("out/pcl_{}", pcl_cnt);
                                    pcl.save_to_file(&fp);
                                    pcl_cnt += 1;
                                }
                            }


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
