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
use std::io::{Read, stdin, Write};
use std::process::exit;
use std::net::UdpSocket;
use std::sync::mpsc;
use std::{any, fs};

use std::time::{SystemTime, Duration};
use std::ops::{Index, Mul};
use std::{thread};

use std::sync::mpsc::{Receiver, Sender};




use std::sync::{Arc, Mutex};


use std::fs::OpenOptions;
use tracing::{info, error};

const SERIAL : bool = false;


pub struct SystemController{
    ///The camera objects that are plugged in 
    cameras : Vec<Arc<Mutex<CamType>>>,
    ///The current heightmap of the system
    global_hmap : Heightmap,

    //Current position (tcp mm from the base of the robot)
    curr_pos : [f32; 3],
    //Current orientation (tcp quartenion from the base of the robot)
    curr_ori : [f32; 4]
}


//Filepath for height map saving
const HMAP_FP : &str = "/home/trl/Desktop/global";

//Filepath for cam pos estimation log
const CAM_LOG_FP : &str = "../../out/cam_pos_log.txt";


//Heightmap size controllers - hmap size based on a resolution of 0.0010m over a space of 1m?
const HMAP_RES : f32 = 0.001;
const GLOBAL_AREA_WIDTH : f32 = 1.0;
const GLOBAL_AREA_HEIGHT : f32 = 1.0;


const GLOBAL_HMAP_WIDTH : usize = (GLOBAL_AREA_WIDTH / HMAP_RES) as usize;
const GLOBAL_HMAP_HEIGHT : usize = (GLOBAL_AREA_HEIGHT / HMAP_RES) as usize;


//CURRENT FIRING ORDER 
//BL->BR->FRONT


//CALIB POS TO WORLD TRANSFORM -----------------

///CALIBRATION POSITION - WITH NO TOOL (i.e. includes force sensor)
const CALIB_POS : [f32;3] = [171.54, 2356.35, 608.81];
const CALIB_ORI : [f32; 4] = [0.00929, 0.16741, -0.98583, 0.00509];


const FRONT_SPOKE_TRANSFORM : Matrix4<f32> = matrix![-0.99899551557681, -0.04226224567962896, -0.014895047752140629, 0.3122061186469016;
0.030435006218420737, -0.8839017726881305, 0.4666812259297188, -0.03251663667695311;
-0.03288875573670366, 0.465759121036717, 0.8843001588358994, -0.5960938172876132;
0.0, 0.0, 0.0, 1.0;];





const BACK_SPOKE_L_TRANSFORM : Matrix4<f32> = matrix![0.48159271263130415, -0.8008477335837144, 0.35596540106343183, 0.02033132736153277;
0.8748503520584744, 0.4151969625398048, -0.2494961799324453, 0.3991844896418486;
0.05201269694589321, 0.4315719985257937, 0.9005777531367666, -0.6066500545047026;
0.0, 0.0, 0.0, 1.0;];



const BACK_SPOKE_R_TRANSFORM : Matrix4<f32> = matrix![0.4920263818185973, 0.7670088131415131, -0.4118343358168998, 0.5509871548626446;
-0.8704340799437051, 0.44208750975294203, -0.2165713420399797, 0.44157190844299665;
0.015954687933500567, 0.46503345501553917, 0.8851493284464802, -0.5877192219740137;
0.0, 0.0, 0.0, 1.0;];




const T_WORLD_CALIB : [Matrix4<f32>; 3] = [BACK_SPOKE_L_TRANSFORM , BACK_SPOKE_R_TRANSFORM, FRONT_SPOKE_TRANSFORM];





///FORCE SENSOR TO CAMERA TRANSFORMS - DEFINED IN THE TCP FRAME -opencv calced

const FORCE_TO_FRONT_CAM : Matrix4<f32> = matrix![0.9464713852548146, -0.18468624457298505, -0.26473176605643794, -0.28688827255775634;
0.1002264109807594, 0.9477518654295058, -0.30285486312226784, 0.20644821188555162;
-0.30683315243915377, -0.2601103470460555, -0.9155304604019643, 0.7603868110360974;
0.0, 0.0, 0.0, 1.0;];


const FORCE_TO_BL_CAM: Matrix4<f32> = matrix![-0.2553413480909949, 0.8838008918456687, -0.39204180839277014, 0.2117304810346405;
-0.9667589409859791, -0.23898039925061482, 0.09091489865628377, -0.0365676482947881;
-0.013339639378156284, 0.4022242562884742, 0.9154439915551602, -0.01916256485698814;
0.0, 0.0, 0.0, 1.0;];



const FORCE_TO_BR_CAM : Matrix4<f32> = matrix![0.6767664387744544, 0.6539795790193381, 0.3380797207379216, -0.19565337878514497;
-0.7355242093078449, 0.620275285946706, 0.2725118477530901, -0.11913406185413351;
-0.031485311982148545, -0.4330926920064818, 0.9007993090916345, -0.11452860657587213;
0.0, 0.0, 0.0, 1.0;];


const T_LC_CAM :[Matrix4<f32>; 3] = [FORCE_TO_BL_CAM, FORCE_TO_BR_CAM, FORCE_TO_FRONT_CAM];


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

const FINE_TUNES:[Matrix4<f32>; 3] = [BL_FINE_TUNE, BR_FINE_TUNE, FRONT_FINE_TUNE];



//Default croppings for each camera
const FRONT_SPOKE_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];
const BACK_SPOKE_L_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];
const BACK_SPOKE_R_CROP : [f32;6] = [-0.5, 0.5, -0.5, 0.5, -999.0, 999.0];

const CROP_LIST : [[f32;6];3] = [BACK_SPOKE_L_CROP, BACK_SPOKE_R_CROP, FRONT_SPOKE_CROP];


impl SystemController{

    ///Starts the system, taking control away from the user
    pub fn start_system_control(config : &mut ConfigManager) -> Result<Self, anyhow::Error>{

      

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

        //Create the cam_move log
        //Create the data file with the csv header
        let mut data_file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(CAM_LOG_FP)
            .unwrap();


        //If valid cameras - 
        println!(">Getting camera list...");
        let cam_list = config.cams();
        let mut connected_cams : Vec<Arc<Mutex<CamType>>> = vec![];

        let mut realsense_cnt = 0;
        let mut zed_cnt = 0;

    
        

        println!(">Iterating over camera list...");
        for cam in cam_list{
            if cam == "Realsense"{
                println!("Realsense cnt: {}", realsense_cnt);
                //Always connect to 0th camera because 
                connected_cams.push(Arc::new(Mutex::new(CamType::RealsenseCam(DepthCam::connect_realsense(0)?))));
                realsense_cnt += 1;
                println!(">Realsense camera connected");
            }
            if cam == "STEREOLABS"{
                connected_cams.push(Arc::new(Mutex::new(CamType::ZedCamera(DepthCam::connect_zed(zed_cnt)?))));
                zed_cnt += 1;
                println!(">Zed camera connected");
            }
        }

        

        info!("{} realsense cams detected", realsense_cnt);
        info!("{} zed cams detected", zed_cnt);

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

    pub fn set_pos(&mut self, pos : [f32;3]){
        self.curr_pos = pos;
    }

    pub fn set_ori(&mut self, ori : [f32;4]){
        self.curr_ori = ori;
    }


    ///Fire all of the depth cameras the system controls
    pub fn fire_all_cams(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{

        info!("Cams firing");
    
        let mut pcl_vec : Vec<PointCloud> = vec![];

        for (i, cam) in self.cameras.iter_mut().enumerate(){  
            
            let mut cam = cam.lock().unwrap();

            pcl_vec.push(cam.take_pcl()?);
        }
     
        info!("Cams fired");

        Ok(pcl_vec)
    }

    ///Fire all of the cameras asynchronously in seperate tokio tasks
    pub fn async_fire_all(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{

        //println!("Asynchronously firing all");

        let results: Vec<Result<PointCloud, anyhow::Error>> = std::thread::scope(|s| {
            let handles: Vec<_> = self
                .cameras
                .iter()
                .map(|cam| {
                    s.spawn(move || -> Result<PointCloud, anyhow::Error> {
                        let mut cam = cam
                            .lock()
                            .map_err(|e| anyhow::anyhow!("Failed to get cam lock - {e}"))?;
                        //println!("Acquired cam lock");
                        cam.take_pcl()
                    })
                })
                .collect();

            handles
                .into_iter()
                .map(|h| h.join().unwrap_or_else(|_| Err(anyhow::anyhow!("Thread panicked"))))
                .collect()
        });

        let mut pcls = Vec::new();
        for res in results {
            match res {
                Ok(pcl) => {
                    //println!("pcl captured");
                    pcls.push(pcl);
                }
                Err(e) => println!("Capture error! - {e}"),
            }
        }
        Ok(pcls)

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
        
            let cam_at_calib =   tcp_at_calib *  T_NOTOOL_LC   *  T_LC_CAM[i];


            


            //Calculate the cameras current position
            let curr_pos_m = Vector3::new(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2]) / 1000.0;
            let q_curr = UnitQuaternion::from_quaternion(
                Quaternion::new(self.curr_ori[0], self.curr_ori[1], self.curr_ori[2], self.curr_ori[3])
            );


            let tcp_at_curr = Translation3::from(curr_pos_m).to_homogeneous() * q_curr.to_homogeneous();
            let cam_at_curr = tcp_at_curr *  T_NOTOOL_LC  *  T_LC_CAM[i];

            //Calculate the transformation from the calibration frame to the current camera frame
            let T_calib_curr= cam_at_calib.try_inverse().unwrap() * cam_at_curr;
            


            //println!("cam delta to calibration cam pos: {}", T_calib_curr);

            //--------------------
        
            //The point is transformed from the current camera space -> calibration camera space -> world space
            //The camera space is calculated by doing a rigid transformation from the tcp position/orientation to the position of the camera
        

            let T_world_curr =   /*FINE_TUNES[i]*/ T_WORLD_CALIB[i] * T_calib_curr;

            //println!("Final transform: {}", T_world_curr);


            pcl.transform_with(&T_world_curr);         

            

        }

        info!("Pointclouds transformed")

    }


    ///Runs the autonomous mapping control loop
    pub fn auto_map_start(&mut self) -> Result<(), anyhow::Error>{
        println!(">automapping start - WARNING - DO NOT TYPE");

        info!("Auto mapping started");

        const PCL_DEBUG :bool = false;
        let mut pcl_cnt = 0;

        const HMAP_DEBUG : bool = false;
        let mut hmap_cnt = 0;


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
                            let mut pcl_list = if SERIAL{
                                
                                self.fire_all_cams()?
                            }else{

                                self.async_fire_all()?
                            };


                            //Crop the point cloud
                            self.standard_crop(&mut pcl_list);

                            //Go through each point cloud and transform it to the work space
                            self.workspace_transform(&mut pcl_list);    

                            if PCL_DEBUG{
                                for pcl in &pcl_list{
                                    let fp = format!("../../out/pcl_{}", pcl_cnt);
                                    pcl.save_to_file(&fp);
                                    pcl_cnt += 1;
                                }
                            }


                            //Group the pointclouds and turn them into a heightmap - resolution based on desired resolution
                            let local_hmap = Heightmap::create_from_pcl_list_with_res(pcl_list, HMAP_RES)?;
                            info!("Local heightmap created");;

                            if HMAP_DEBUG{
                                let fp = format!("../../out/hmap_{}", hmap_cnt);
                                    local_hmap.save_to_file(&fp);
                                    hmap_cnt += 1;
                            }

                            
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

                            self.log_cam_pos_est();
                            

                        };  
                    }     
            
                    
                }
            }

            Ok(())
        }

    
    ///A loop that constantly saves pointclouds - without transforming them
    pub fn pcl_save_loop(&mut self) -> Result<(), anyhow::Error>{


        const POINTCLOUD_LIMIT : usize = 1000;

        println!("Please provide a test name");

        //Get the user filepath
        let mut inp= String::new();
        stdin().read_line(&mut inp).expect("Did not enter a correct string");

        let base_fp = format!("../../out/{}", inp.strip_suffix("\n").unwrap());

        let mut total = 0;

        //Maximum 1000 pointclouds
        for i in 0..POINTCLOUD_LIMIT{

            //Take the pointclodus asynchronously
            let pcls = self.async_fire_all()?;


            //Save each pointcloud with a unique name
            for (j, pcl) in pcls.iter().enumerate(){
                let fp = format!("{}_{}_{}", base_fp, total, j);
                pcl.save_to_file(&fp);
            }

            total += 1;

            println!("{} pcl set saved", total);
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
    pub fn get_all_intrinsics(&self) -> Result<Vec<IntrinsicInfo>, anyhow::Error>{

        //Request the intrinsic matrix info from the camera
        let mut intrinsics : Vec<IntrinsicInfo> = vec![];

        for cam in self.cameras.iter(){
            let cam = cam.lock().unwrap();
            intrinsics.push(cam.get_intrinsics()?)
        }

        Ok(intrinsics)
    }

    ///Get every camera to take an rgbd image
    pub fn fire_all_cams_image(&mut self, base_filepath : &str) -> Result<Vec<String>, anyhow::Error>{
        
        let mut filepaths : Vec<String> = vec![];
        
        //Create each image and label it according to its number in the id
        for (i, cam) in self.cameras.iter_mut().enumerate(){

            let mut cam = cam.lock().unwrap();

            println!("Cam {} firing", i);   
            let img_fp = format!("{}_{}", base_filepath, i);
            println!("{}", img_fp);                     
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



    ///Get the estimated position from every single camera
    pub fn get_pos(&mut self) -> Result<Vec<[f32;3]>, anyhow::Error>{

        let mut pos_vec : Vec<[f32;3]> = vec![];

        for cam in self.cameras.iter_mut(){

            let mut cam = cam.lock().unwrap();

            //Check that the cameras imu is on
            if cam.imu_enabled(){
                pos_vec.push(cam.get_pos()?);
            } 
        }
        return Ok(pos_vec)
    }

    ///Log the camera reported position
    pub fn log_cam_pos_est(&mut self) -> Result<(), anyhow::Error>{

        let pos_ests = self.get_pos()?;

        //Create the data file with the csv header
        let mut data_file = OpenOptions::new()
            .append(true)
            .open(CAM_LOG_FP)
            .unwrap();

        data_file.write_all(format!("{:?}\n", pos_ests).as_bytes());


        Ok(())
    }


    ///Reset all connected hardware
    pub fn reset_hardware() -> Result<(), anyhow::Error>{
        
        
        RealsenseCam::reset_all();
        info!("Cameras reset");
        Ok(())
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
