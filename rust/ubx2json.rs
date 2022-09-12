use std::io;
use std::io::Read;

fn main() {
    let reader: Box<dyn Read> = Box::new(io::stdin());
    let mut parser = ublox::Parser::default();
    let data: Vec<u8> = reader.bytes().map(|a| a.unwrap()).collect();
    let mut it = parser.consume(&data);
    loop {
        match it.next() {
            Some(Ok(packet)) => {
                let json = serde_json::to_string(&packet).unwrap();
                println!("{}", json);
            }
            Some(Err(_)) => {}
            None => break,
        }
    }
}
