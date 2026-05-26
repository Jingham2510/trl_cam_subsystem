///Entry point for anything that runs the package

use std::io::stdin;
use std::time::SystemTime;
use chrono::DateTime;
use chrono::offset::Local;


///Entry point - presents the homepage
fn main() {
    println!(".___________. _______ .______      .______          ___           .______        ______   .______     ______   .___________. __    ______     _______.    __          ___      .______ ");
    println!("|           ||   ____||   _  \\     |   _  \\        /   \\          |   _  \\      /  __  \\  |   _  \\   /  __  \\  |           ||  |  /      |   /       |   |  |        /   \\     |   _  \\  ");
    println!("`---|  |----`|  |__   |  |_)  |    |  |_)  |      /  ^  \\   ______|  |_)  |    |  |  |  | |  |_)  | |  |  |  | `---|  |----`|  | |  ,----'  |   (----`   |  |       /  ^  \\    |  |_)  | ");
    println!("    |  |     |   __|  |      /     |      /      /  /_\\  \\ |______|      /     |  |  |  | |   _  <  |  |  |  |     |  |     |  | |  |        \\   \\       |  |      /  /_\\  \\   |   _  <  ");
    println!("    |  |     |  |____ |  |\\  \\----.|  |\\  \\----./  _____  \\       |  |\\  \\----.|  `--'  | |  |_)  | |  `--'  |     |  |     |  | |  `----.----)   |      |  `----./  _____  \\  |  |_)  | ");
    println!("    |__|     |_______|| _| `._____|| _| `._____/__/     \\__\\      | _| `._____| \\______/  |______/   \\______/      |__|     |__|  \\______|_______/       |_______/__/     \\__\\ |______/  ");



    println!("                                                _                     _                 ");
    println!("                                               | |                   | |                ");
    println!("  ___ __ _ _ __ ___   ___ _ __ __ _   ___ _   _| |__    ___ _   _ ___| |_ ___ _ __ ___  ");
    println!(" / __/ _` | '_ ` _ \\ / _ \\ '__/ _` | / __| | | | '_ \\  / __| | | / __| __/ _ \\ '_ ` _ \\ ");
    println!("| (_| (_| | | | | | |  __/ | | (_| | \\__ \\ |_| | |_) | \\__ \\ |_| \\__ \\ ||  __/ | | | | |");
    println!(" \\___\\__,_|_| |_| |_|\\___|_|  \\__,_| |___/\\__,_|_.__/  |___/\\__, |___/\\__\\___|_| |_| |_|");
    println!("                                                            __/ |                      ");
    println!("                                                            |___/                       ");

    let system_time = SystemTime::now();
    let datetime: DateTime<Local> = system_time.into();

    println!("The systems date is currently: {}", datetime.format("%d/%m/%Y"));
    println!("The systems time is currently: {}", datetime.format("%T"));

    command_handler()
}



///Handles all input and command from the user
fn command_handler(){

    //List of valid commands - that the user is told about
    const CMD_LIST : [&str; 2] = ["help  - lists valid commands", "quit - closes the program"];

    println!("Awaiting user input...");

    loop{

    let mut inp= String::new();

    stdin().read_line(&mut inp).expect("Did not enter a correct string");


    match inp.to_lowercase().trim(){
        "help" => {
            for cmd in CMD_LIST{
                println!(">{}", cmd);
            }
        }

        //Quit the program
        "quit" => {
            return;
        }

        _ => {println!(">Invalid command - see 'help' for a list of available commands")}

    }
}

    


  

}