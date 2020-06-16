// Copy this into the decoded function in your TTN application.

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};
  
    decoded.avg_im = (bytes[1] << 8) + bytes[0];
    decoded.min_im = (bytes[3] << 8) + bytes[2];
    decoded.max_im = (bytes[5] << 8) + bytes[4];
    decoded.avg_ex = (bytes[7] << 8) + bytes[6];
    decoded.min_ex = (bytes[9] << 8) + bytes[8];
    decoded.max_ex = (bytes[11] << 8) + bytes[10];
    decoded.num_frames = (bytes[13] << 8) + bytes[12];
    
  
    return decoded;
  }