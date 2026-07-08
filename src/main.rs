///Entry point for anything that runs the package

use std::io::stdin;
use std::time::SystemTime;
use chrono::DateTime;
use chrono::offset::Local;
use std::env;

mod config;
mod sys_cntrl;

use crate::config::valid_cams::{VENDOR_IDS, VENDOR_NAMES};
use crate::config::config_manager::ConfigManager;
use crate::sys_cntrl::system_control::SystemController;




///Entry point - presents the homepage
fn main() {

    unsafe{
    env::set_var("RUST_BACKTRACE", "1");
    }

    println!(">Starting config manager");
    let config_manager = ConfigManager::start_manager();
    println!(">Config manager started");
    println!(">{} device(s) detected", config_manager.no_of_cams());


    println!(">.___________. _______ .______      .______          ___           .______        ______   .______     ______   .___________. __    ______     _______.    __          ___      .______ ");
    println!(">|           ||   ____||   _  \\     |   _  \\        /   \\          |   _  \\      /  __  \\  |   _  \\   /  __  \\  |           ||  |  /      |   /       |   |  |        /   \\     |   _  \\  ");
    println!(">`---|  |----`|  |__   |  |_)  |    |  |_)  |      /  ^  \\   ______|  |_)  |    |  |  |  | |  |_)  | |  |  |  | `---|  |----`|  | |  ,----'  |   (----`   |  |       /  ^  \\    |  |_)  | ");
    println!(">    |  |     |   __|  |      /     |      /      /  /_\\  \\ |______|      /     |  |  |  | |   _  <  |  |  |  |     |  |     |  | |  |        \\   \\       |  |      /  /_\\  \\   |   _  <  ");
    println!(">    |  |     |  |____ |  |\\  \\----.|  |\\  \\----./  _____  \\       |  |\\  \\----.|  `--'  | |  |_)  | |  `--'  |     |  |     |  | |  `----.----)   |      |  `----./  _____  \\  |  |_)  | ");
    println!(">    |__|     |_______|| _| `._____|| _| `._____/__/     \\__\\      | _| `._____| \\______/  |______/   \\______/      |__|     |__|  \\______|_______/       |_______/__/     \\__\\ |______/  ");



    println!(">                                                _                     _                 ");
    println!(">                                               | |                   | |                ");
    println!(">  ___ __ _ _ __ ___   ___ _ __ __ _   ___ _   _| |__    ___ _   _ ___| |_ ___ _ __ ___  ");
    println!("> / __/ _` | '_ ` _ \\ / _ \\ '__/ _` | / __| | | | '_ \\  / __| | | / __| __/ _ \\ '_ ` _ \\ ");
    println!(">| (_| (_| | | | | | |  __/ | | (_| | \\__ \\ |_| | |_) | \\__ \\ |_| \\__ \\ ||  __/ | | | | |");
    println!("> \\___\\__,_|_| |_| |_|\\___|_|  \\__,_| |___/\\__,_|_.__/  |___/\\__, |___/\\__\\___|_| |_| |_|");
    println!(">                                                            __/ |                      ");
    println!(">                                                            |___/                       ");

    let system_time = SystemTime::now();
    let datetime: DateTime<Local> = system_time.into();

    println!(">The systems date is currently: {}", datetime.format("%d/%m/%Y"));
    println!(">The systems time is currently: {}", datetime.format("%T"));

    command_handler(config_manager)
}



///Handles all input and command from the user
fn command_handler(mut config_manager : ConfigManager){

    //Get the cmd line arguments
    let args: Vec<String> = env::args().collect();


    if args.len() > 1{
        //If the first argument is "auto" - assume main system control
        if args[1] == "-auto"{
            match SystemController::start_system_control(&mut config_manager){

                Ok(mut sys_cntrller) =>{
                    println!(">Controller started");
                    
                    sys_cntrller.auto_map_start();

                    println!(">Controller finished");

                }
                Err(e) =>{
                    println!("{e}");
                    println!(">Exiting system control")
                }
            }
            return           
        }
    }


    //If any of the arguments read "auto" instantly go into system auto mode

    //List of valid commands - that the user is told about
    const CMD_LIST : [&str; 5] = ["help  - lists valid commands", "quit - closes the program", "time - displays systems date and time", "lsusb - show connected usb RGBD cameras", "start system - Start the system controller (removes user input)"];


    //Do the command handler forever
    loop{

        let mut inp= String::new();

        stdin().read_line(&mut inp).expect("Did not enter a correct string");


        match inp.to_lowercase().trim(){
            //Print all public commands
            "help" => {
                for cmd in CMD_LIST{
                    println!(">{}", cmd);
                }
            }
            //Display date and time
            "time" => {
                let system_time = SystemTime::now();
                let datetime: DateTime<Local> = system_time.into();

                println!(">The systems date is currently: {}", datetime.format("%d/%m/%Y"));
                println!(">The systems time is currently: {}", datetime.format("%T"));
            }

            //Quit the program
            "quit" => {
                println!(">closing system");
                return;
            }

            //Show the connected usb RGBD cameras
            "lsusb" =>{
    
                let mut valid_devs = false;

                for device in rusb::devices().unwrap().iter() {
                    let device_desc = device.device_descriptor().unwrap();

                    if VENDOR_IDS.contains(&device_desc.vendor_id()){
                        valid_devs = true;
                        println!(">{} - Bus {:03} Device {:03} ID {:04x}:{:04x}",
                        VENDOR_NAMES[VENDOR_IDS.iter().position(|&r| r == device_desc.vendor_id()).unwrap()],
                        device.bus_number(),
                        device.address(),
                        device_desc.vendor_id(),
                        device_desc.product_id());
                    }                
                }

                if !valid_devs{
                    println!(">No valid cameras detected");
                }

            }

            //Updates the config manager
            "config" =>{
                println!(">Updating config");
                config_manager.update();
                println!(">Config updated");
                println!(">{} device(s) detected", config_manager.no_of_cams());

            }

            //Start the system controller 
            "start system" =>{
                

                match SystemController::start_system_control(&mut config_manager){

                    Ok(mut sys_cntrller) =>{
                        println!(">Controller started");
                        println!("{:?}", sys_cntrller.auto_map_start());



                    }
                    Err(e) =>{
                        println!("{e}");
                        println!(">Exiting system control")
                }

                }
            }

            //Take photos with each rgbd camera
            "take pic" =>{


                match SystemController::start_system_control(&mut config_manager){

                    Ok(mut sys_cntrller) =>{
                        println!(">System started");
                        
                        println!("{:?}", sys_cntrller.fire_all_cams_image("out/fired"));



                    }
                    Err(e) =>{
                        println!("{e}");
                        println!(">Exiting system control")
                        }
                }
            }

            //Get extrinsics of the currently plugged in cameras
            "get extrinsics" =>{
                  match SystemController::start_system_control(&mut config_manager){

                    Ok(mut sys_cntrller) =>{
                        println!(">Getting extrinsics");

                        println!("{:?}", sys_cntrller.calc_calib_mats(false));

                    }
                    Err(e) =>{
                        println!("{e}");
                        println!(">Exiting system control")
                }

                }
            }

            //Fire all cameras to get pointclouds
            "get pcls" =>{
                match SystemController::start_system_control(&mut config_manager){

                    Ok(mut sys_cntrller) =>{
                        println!("Firing all");

                        let pcls = sys_cntrller.fire_and_transform()?;

                        let mut i = 0;

                        for pcl in pcls{
                            pcl.save_to_file(&format!("out/pcl_{i}"));
			                i += 1;
                        }



                    }
                    Err(e) =>{
                        println!("{e}");
                        println!(">Exiting system control");
                    }
                }
            }

            //Catch all invalid/un=implemented commands
            _ => {println!(">Invalid command - see 'help' for a list of available commands")}

    }
}

    


  

}
