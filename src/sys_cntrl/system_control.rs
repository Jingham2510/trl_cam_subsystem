//!Controls the system when in autonomous mode - i.e. Robot moving and heightmap being generated
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;
use rustgeomapping::depth_cam::{CamType, DepthCam};


use rustgeomapping::data_types::heightmap::Heightmap;
use rustgeomapping::data_types::pointcloud::PointCloud;
use rustgeomapping::data_types::intrinsic_info::IntrinsicInfo;
use rustgeomapping::computer_vision::{get_extrinsic_inv_from_aruco_4x4_250, get_extrinsic_inv_from_board};
use crate::config::config_manager::ConfigManager;
use anyhow::bail;
use nalgebra::{UnitQuaternion, Quaternion, Vector3, Matrix4, Translation3, matrix};

use std::cell::RefCell;
use std::io::{Read, stdin};
use std::process::exit;
use std::net::UdpSocket;
use std::sync::mpsc;
use std::{any, fs};
use std::fs::OpenOptions;
use std::time::{SystemTime, Duration};
use std::ops::{Index, Mul};
use std::{thread};

use std::sync::mpsc::{Receiver, Sender};

//use crate::sys_cntrl::cam_thread::CamThread;


use tracing::{info, error};

const SERIAL : bool = true;


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


//Heightmap size controllers - hmap size based on a resolution of 0.0010m over a space of 1m?
const HMAP_RES : f32 = 0.001;
const GLOBAL_AREA_WIDTH : f32 = 1.0;
const GLOBAL_AREA_HEIGHT : f32 = 1.0;


const GLOBAL_HMAP_WIDTH : usize = (GLOBAL_AREA_WIDTH / HMAP_RES) as usize;
const GLOBAL_HMAP_HEIGHT : usize = (GLOBAL_AREA_HEIGHT / HMAP_RES) as usize;





//CALIB POS TO WORLD TRANSFORM -----------------

///NEW TRANSFORMS
const CALIB_POS : [f32;3] = [121.04, 2446.29, 248.80];
const CALIB_ORI : [f32; 4] = [0.00317, -0.17772, 0.98399, 0.01286];

//Added 5cm of height to each transform to properly place in the sandbed

const FRONT_SPOKE_TRANSFORM : Matrix4<f32> = matrix![ 0.9998314,  -0.00798137, -0.01653686,  0.22178532;
                                                     0.01464439,  0.88990805,  0.45590481, -0.09734951;
                                                     0.01107754, -0.45607012,  0.8898749,  -0.62943091;
                                                     0.        ,  0.        ,  0.       ,   1.        ];





const BACK_SPOKE_L_TRANSFORM : Matrix4<f32> = matrix![ 0.44983659, -0.80929274,  0.37774636, -0.03133563;
                                                     0.89303276,  0.40198995, -0.20222899,  0.32980309;
                                                     0.01181221,  0.42830987,  0.90355472, -0.63459977;
                                                     0.        ,  0.        ,  0.        ,  1.        ];




const BACK_SPOKE_R_TRANSFORM : Matrix4<f32> = matrix![ 0.50649045,  0.77714794, -0.37350837,  0.49473645;
                                                    -0.86224487,  0.45595135, -0.22054965,  0.39169167;
                                                    -0.00109806,  0.43376197,  0.90102683, -0.61863551;
                                                     0.        ,  0.        ,  0.        ,  1.        ];

const T_WORLD_CALIB : [Matrix4<f32>; 3] = [FRONT_SPOKE_TRANSFORM , BACK_SPOKE_L_TRANSFORM , BACK_SPOKE_R_TRANSFORM];





///FORCE SENSOR TO CAMERA TRANSFORMS - DEFINED IN THE TCP FRAME -opencv calced

const FORCE_TO_FRONT_CAM : Matrix4<f32> = matrix![-0.9420656824831528, 0.3011015127512243, 0.147817891022961, -0.0661588662273698;
                                                -0.3353728941901005, -0.8374960703056575, -0.431422477468591, 0.31183874579145265;
                                                -0.006105057748074003, -0.4560024246004681, 0.8899575928258532, -0.07290399838878403;
                                                0.0, 0.0, 0.0, 1.0;];


const FORCE_TO_BL_CAM: Matrix4<f32> = matrix![-0.12353354300547559, 0.8971388147126936, -0.42412428707681615, 0.31568317389970685;
                                            -0.9923333518892358, -0.113291896393938, 0.0493909398547879, -0.008723076643095948;
                                            -0.003739315550789185, 0.42697411320520173, 0.9042561164691356, -0.08104667759030465;
                                            0.0, 0.0, 0.0, 1.0;];



const FORCE_TO_BR_CAM : Matrix4<f32> = matrix![-0.7093846891906819, -0.647396861303359, 0.2786586921601385, -0.1703756814912174;
                                            0.7048130123989124, -0.6496388541232674, 0.2849701331132417, -0.23776528199520605;
                                            -0.0034612562763123578, 0.39855572155969266, 0.9171375886512465, -0.06462090766642739;
                                            0.0, 0.0, 0.0, 1.0;];


const T_LC_CAM :[Matrix4<f32>; 3] = [FORCE_TO_FRONT_CAM, FORCE_TO_BL_CAM, FORCE_TO_BR_CAM];


///TCP POINT TO FORCE SENSOR POINT TRANSFORM - DEFINED IN THE CURRENT TCP FRAME
/// MEASURE FROM THE FORCE SENSOR TO TCP DO NOT MATCH THE TCP SPECIFIED IN RAPID
const T_STCP_LC : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                                            0.0, 1.0, 0.0, 0.0;
                                                            0.0, 0.0, 1.0, 0.45;
                                                            0.0, 0.0, 0.0, 1.0];

const T_NOTOOL_LC: Matrix4<f32> =  matrix![1.0, 0.0, 0.0, 0.0;
                                                            0.0, 1.0, 0.0, 0.0;
                                                            0.0, 0.0, 1.0, 0.0;
                                                            0.0, 0.0, 0.0, 1.0];



///Hand-fine tuned last touches - calibed to the front pointcloud
const FRONT_FINE_TUNE : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                               0.0, 1.0, 0.0, 0.0;
                                               0.0, 0.0, 1.0, 0.0;
                                               0.0, 0.0, 0.0, 1.0];

const BL_FINE_TUNE : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                               0.0, 1.0, 0.0, 0.0;
                                               0.0, 0.0, 1.0, 0.0;
                                               0.0, 0.0, 0.0, 1.0];

const BR_FINE_TUNE : Matrix4<f32> = matrix![1.0, 0.0, 0.0, 0.0;
                                               0.0, 1.0, 0.0, 0.0;
                                               0.0, 0.0, 1.0, 0.0;
                                               0.0, 0.0, 0.0, 1.0];

const FINE_TUNES:[Matrix4<f32>; 3] = [FRONT_FINE_TUNE, BL_FINE_TUNE, BR_FINE_TUNE];



//Default croppings for each camera
const FRONT_SPOKE_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];
const BACK_SPOKE_L_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];
const BACK_SPOKE_R_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];

const CROP_LIST : [[f32;6];3] = [FRONT_SPOKE_CROP, BACK_SPOKE_L_CROP, BACK_SPOKE_R_CROP];


impl SystemController{

    ///Starts the system, taking control away from the user
    pub fn start_system_control(config : &mut ConfigManager) -> Result<Self, anyhow::Error>{

        //Create a blank logging file in the output (or overwrite the previous log if it exists)
        let log_file = OpenOptions::new()
            .create(true)
            .write(true)
            .open("../../out/log.log")
            .unwrap();

        //Start the logging tool
        tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .with_target(false)
        .with_writer(log_file)
        .init();

        info!("Logging started");

        println!(">Starting system control - no longer accepting typed user input");
        println!(">GLOBAL WIDTH:{} GLOBAL HEIGHT:{}", GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_WIDTH);

        //Update the config - not required by the system as it won't be updated while the system is alive
        config.update();

        info!("Config updated");

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

        info!("{} cams detected", realsense_cnt);

        let mut global_hmap = Heightmap::new(GLOBAL_HMAP_WIDTH, GLOBAL_HMAP_HEIGHT);
        global_hmap.set_lower_coord_bounds([0.0, 0.0]);
        global_hmap.set_upper_coord_bounds([1.0, 1.0]);
        global_hmap.set_all_cells(f32::NAN);

        info!("Global heightmap created");


        Ok(SystemController{
            cameras : connected_cams,
            global_hmap,
            curr_pos : [0.0, 0.0, 0.0],
            curr_ori : [0.0, 0.0, 0.0, 0.0]
        })
    }


    ///Fire all of the depth cameras the system controls and saves the pointclouds
    pub fn fire_all_cams(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{

        info!("Cams firing");
    
        let mut pcl_vec : Vec<PointCloud> = vec![];

        for (i, cam) in self.cameras.iter_mut().enumerate(){  
            
            pcl_vec.push(cam.take_pcl()?);
        }
     
        info!("Cams fired");

        Ok(pcl_vec)
    }

    ///Fire all the cameras and perform the workplace transform on each of them
    pub fn fire_and_transform(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{



        let mut pcl_vec = self.fire_all_cams()?;

        self.standard_crop(&mut pcl_vec);        

        self.workspace_transform(&mut pcl_vec);      


     
        Ok(pcl_vec)


    }

    ///Performs the default crop on a list of pointclouds
    fn standard_crop(&self, pcl_list : &mut Vec<PointCloud>){

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){
            let crop = CROP_LIST[i];
            pcl.crop(crop[0], crop[1], crop[2], crop[3], crop[4], crop[5]);
        }

        info!("Pointclouds cropped")

    }

    ///Performs the combined default-workplace transform on a set of pointclouds
    fn workspace_transform(&self, pcl_list : &mut Vec<PointCloud>){

        for (i ,pcl) in pcl_list.iter_mut().enumerate(){  

    

            // Positions in metres
            let calib_pos_m = Vector3::new(CALIB_POS[0], CALIB_POS[1], CALIB_POS[2]) / 1000.0;
            //Create the original quaternion
            let q_calib = UnitQuaternion::from_quaternion(
                Quaternion::new(CALIB_ORI[0], CALIB_ORI[1], CALIB_ORI[2], CALIB_ORI[3])
            );

            let tcp_at_calib = Translation3::from(calib_pos_m).to_homogeneous() * q_calib.to_homogeneous();
        
            let cam_at_calib =   tcp_at_calib *  T_STCP_LC   *  T_LC_CAM[i];


            


            //Calculate the cameras current position
            let curr_pos_m = Vector3::new(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2]) / 1000.0;
            let q_curr = UnitQuaternion::from_quaternion(
                Quaternion::new(self.curr_ori[0], self.curr_ori[1], self.curr_ori[2], self.curr_ori[3])
            );


            let tcp_at_curr = Translation3::from(curr_pos_m).to_homogeneous() * q_curr.to_homogeneous();
            let cam_at_curr = tcp_at_curr *  T_STCP_LC  *  T_LC_CAM[i];

            //Calculate the transformation from the calibration frame to the current camera frame
            let T_calib_curr= cam_at_calib.try_inverse().unwrap() * cam_at_curr;
            


            //println!("cam delta to calibration cam pos: {}", T_calib_curr);

            //--------------------
        
            //The point is transformed from the current camera space -> calibration camera space -> world space
            //The camera space is calculated by doing a rigid transformation from the tcp position/orientation to the position of the camera
        

            let T_world_curr =   FINE_TUNES[i] * T_WORLD_CALIB[i] * T_calib_curr;

            //println!("Final transform: {}", T_world_curr);


            pcl.transform_with(&T_world_curr);         

            

        }

        info!("Pointclouds transformed")

    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - WARNING - DO NOT TYPE");

        /*                
        //If non-serial mode create the camera threads
        let (threads, triggers, outs) : Option<(Vec<CamThread>, Vec<Sender<bool>>, Vec<Receiver<PointCloud>>)> = if !SERIAL{

            let threads : Vec<CamThread> = vec![];
            let triggers : Vec<Sender<bool>> = vec![];
            let outs : Vec<Receiver<PointCloud>> = vec![];

            for cam in self.cameras{

                let new_trigger : (Sender<bool>, Receiver<bool>) = mpsc::channel();
                let new_out : (Sender<PointCloud>, Receiver<PointCloud>) = mpsc::channel();

                triggers.push(new_trigger.0);
                outs.push(new_out.1);

                

                threads.append(CamThread::prepare(RefCell::new(cam), &cam.id(), new_trigger.0, new_out.1));
            }

            (threads, triggers, outs).into()
        }else{
            //Otherwise just create a bunch of empty vectors that will go unused (probably inefficient)
            Option::None
        };
        */


        info!("Auto mapping started");

        const PCL_DEBUG :bool = false;
        let mut pcl_cnt = 0;


        //Create a new network listener
        let mut stream = UdpSocket::bind("0.0.0.0:8080")?;
        stream.connect("192.168.55.100:8080")?;

        info!("Output socket connected");

        let mut buf : [u8; 10] = [0;10]; 

        loop{
            let n = stream.recv(buf.as_mut_slice())?;

            let inp = str::from_utf8(&buf[..n])?;

            if inp == "CONNECT?"{
                stream.send(b"YES");
                break;
             }
        }

        /*
        if !SERIAL{
            //Turn on the threads if required
            for thread in threads{
                thread.spin_up();
            }
        }*/



        
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
                       
                       /* 
                            let mut pcl_list = if SERIAL{
                                //Fire all cameras
                                self.fire_all_cams()?
                            }else{
                                //Trigger the cameras and wait for each to respond
                                for trigger in triggers{
                                    trigger.send(true);
                                }
                                let mut pcl_list : Vec<PointCloud> = vec![];
                                for out in outs{
                                    pcl_list.push(out.recv())
                                }
                                pcl_list

                            };
*/
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
                            info!("Local heightmap created");;

                            //local_hmap.save_to_file("/home/trl/Desktop/local");
                            
                            //Slot the heightmap into the global heightmap
                            let hmap_updated = self.global_hmap.update_section(local_hmap);

                            match hmap_updated{
                                Ok(_) =>{
                                    info!("Global heightmap updated");
                                }

                                Err(e) =>{
                                    error!("Failed to update heightmap - {}", e)
                                }
                            }

                            


                            //Update the current heightmap file
                            //self.global_hmap.save_to_file(HMAP_FP)?;

                            let flattened_cells = self.global_hmap.get_flattened_cells()?;                            
                            //Turn the list of floats into a list of bytes
                            let bytes : Vec<u8> = flattened_cells.into_iter().flat_map(|i| i.to_be_bytes()).collect();

                            //Tell the main system how many pckets there will be 
                            const PACKET_SIZE : usize = 512;
                            let no_of_packets =bytes.len()/PACKET_SIZE;

                            stream.send(&format!("{}", no_of_packets).into_bytes())?;

                            info!("Number of packets communicated to host pc");

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

                            info!("Packets sent");
                            

                        };  
                    }     
            
                    
                }
            }

            Ok(())
        }



    
    ///Calculate inverse extrinsic matrices based on aruco tag captures
    /// Assumes that the cameras are already in the correct position
    /// Also predefined for the board used in the TRL lab
    pub fn calc_calib_mats(&mut self, delete_calib_imgs : bool, pre_def : bool) -> Result<(), anyhow::Error>{
        
        //For each camera get the intrinsic matrix
        let intrinsics = self.get_all_intrinsics()?;

        //println!("intrinsics: {:?}", intrinsics);

        //Setup the filepath
        let fp = "temp_aruco_calibration";
        //For each camera take a colour image
        let img_filepaths = self.fire_all_cams_image(fp)?; 

        if !pre_def{

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
    }else{

        println!("Predefined board -- (calib.io)");

         for (i, image) in img_filepaths.iter().enumerate(){
            println!(">----------CAM: {}-------------", i);

            match get_extrinsic_inv_from_board(&image, &intrinsics[i]){
                Ok(extrinsic_inv)  =>{                    
                    println!(">-----extrinsics-----");
                    println!(">{}", extrinsic_inv.try_inverse().unwrap());
                    
                    println!(">-----inverse extrinsics-----");
                    println!(">{}", extrinsic_inv);
                }
                Err(e) =>{
                    println!("Error - {e}");
                    println!(">Failed to calc extrinsics for cam");
                }
            }

        

              //If delete is true - delete the images
            if delete_calib_imgs{
                fs::remove_file(image)?;
            }
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
            println!("Cam {} firing", cam.id());   
            let img_fp = format!("{}_{}", base_filepath, cam.id());                     
            filepaths.push(cam.get_colour_image(&img_fp)?);


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


    //Close the system - attempt to disconnect safely - consume self as well
    pub fn close(mut self) -> Result<(), anyhow::Error>{
        
        //Drop the camera resources and hope that the system handles that properly
        for cam in self.cameras{
            drop(cam);
        }

        Ok(())
    }





}
