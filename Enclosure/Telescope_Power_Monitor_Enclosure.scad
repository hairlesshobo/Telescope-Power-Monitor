wallThickness = 3;
topToBottomSplit = 0.3; // percentage of enclosure that is to be used for the top half
extension_x = 8;
extension_y = 20;
extension_z = 5;
standoffHeight = 3;

include <arduino/arduino.scad>

//enclosure(UNO, mountType=PIN);

boardDim = boardDimensions(UNO);

box_x = boardDim[0] + (wallThickness*2) + extension_x;
box_y = boardDim[1] + (wallThickness*2) + extension_y;
box_z = (boardDim[2]*2) + (wallThickness*2) + extension_z;

cavity_x = box_x - (wallThickness*2);
cavity_y = box_y - (wallThickness*2);
cavity_z = box_z - (wallThickness*2);


difference()
{
    // outer shell
    cube ([box_x, box_y, box_z]);
    
    // subtract inner cavity
    translate([wallThickness, wallThickness, wallThickness])
        cube ([cavity_x, cavity_y, cavity_z]);
    
    // remove top
    translate([0, 0, box_z - wallThickness])
        cube([box_x, box_y, wallThickness]);
    
    translate([wallThickness + (extension_x/2), 0, standoffHeight+wallThickness])
    union()
    {
        //standoffs(UNO, mountType = PIN);
        //translate([0, 0, standoffHeight])
        union()
        {
            components(UNO, component = POWER, offset=1);
            components(UNO, component = USB, offset=1);
        }
    }
}

translate([wallThickness + (extension_x/2), wallThickness, wallThickness])
    standoffs(UNO, standoffHeight);

translate([wallThickness + (extension_x/2), wallThickness, standoffHeight + wallThickness])
arduino(UNO);
