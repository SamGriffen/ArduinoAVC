qr_siz = 33; // Size of the QR code

union(){
    //translate([0,0,1.2])import (file = "qr_code.stl", convexity="10");
    qr();
    difference(){   
       cube([55,34.2, 1.2], center=true); // Main base plate
        
        // Holes for mounting
        translate([45/2, 0, 0]){
                translate([0, 24.2/2]){
                    cylinder(1.21, d = 3.2, center = true, $fn = 300);
                    }
                translate([0, -24.2/2]){
                    cylinder(1.21, d = 3.2, center = true, $fn = 300);
                    }
            }
        translate([-45/2, 0, 0]){
                translate([0, 24.2/2]){
                    cylinder(1.21, d = 3.2, center = true, $fn = 300);
                    }
                translate([0, -24.2/2]){
                    cylinder(1.21, d = 3.2, center = true, $fn = 300);
                    }
            }
        }
    }
module qr(){
    translate([-qr_siz/2, -qr_siz/2,1.2]){
        linear_extrude(height = 1.2, center = true, convexity = 10)
        import (file = "qr_code.dxf");
        }
}