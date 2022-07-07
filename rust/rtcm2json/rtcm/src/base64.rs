use serde::{Serialize, Serializer};

pub fn serialize<S: Serializer>(v: &Vec<u8>, s: S) -> Result<S::Ok, S::Error> {
    let base64 = base64::encode(v);
    String::serialize(&base64, s)
}
