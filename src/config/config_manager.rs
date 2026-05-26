//!Keeps track of the config for the system, such that when data is recorded there is a log of what the system state was 
 
use std::time::SystemTime;
use chrono::DateTime;
use chrono::offset::Local;

use crate::config::valid_cams::{VENDOR_IDS, VENDOR_NAMES};

///Writes the configuration of the system parameters
pub struct ConfigManager{
    ///The current date (and time of starting)
    date : DateTime<Local>,
    ///The list of cameras currently connected
    cams : Vec<String>,

    dev_cnt : usize
}

impl ConfigManager{

    ///Starts the manager with current system settings
    pub fn start_manager() -> Self{        
        //Get the current cameras      

        let mut valid_devs = false;
        let mut cams : Vec<String> = vec![];
        let mut dev_cnt = 0;

        for device in rusb::devices().unwrap().iter() {
            let device_desc = device.device_descriptor().unwrap();

            if VENDOR_IDS.contains(&device_desc.vendor_id()){
                valid_devs = true;
                cams.push(format!("{}",
                VENDOR_NAMES[VENDOR_IDS.iter().position(|&r| r == device_desc.vendor_id()).unwrap()]));
                dev_cnt += 1;
            }                
        }

        if !valid_devs{
            cams.push(String::from("None"));
        }



        ConfigManager{
            date: SystemTime::now().into(),
            cams,
            dev_cnt
        }
    }

    ///Update the current config -incase of change of cameras
    pub fn update(&mut self){


        //Get the current cameras      

        let mut valid_devs = false;
        let mut cams : Vec<String> = vec![];
        let mut dev_cnt = 0;
        for device in rusb::devices().unwrap().iter() {
            let device_desc = device.device_descriptor().unwrap();

            if VENDOR_IDS.contains(&device_desc.vendor_id()){
                valid_devs = true;
                cams.push(format!("{}",
                VENDOR_NAMES[VENDOR_IDS.iter().position(|&r| r == device_desc.vendor_id()).unwrap()]));
                dev_cnt += 1;
            }                
        }

        if !valid_devs{
            cams.push(String::from("None"));
        }

        self.date = SystemTime::now().into();
        self.cams = cams;
        self.dev_cnt = dev_cnt;

    }




    pub fn no_of_cams(&self) -> usize{
        self.dev_cnt
    }    
    pub fn cams(&self) ->Vec<String>{
        self.cams.clone()
    }

}


