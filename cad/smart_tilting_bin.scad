/*
 * SMART TILTING BIN - computer-vision refuse sorter
 * Rotating-tilting tray mechanism (first prototype)
 *
 * Mechanical concept
 * ------------------
 *  - Fixed BASE: a central plate holds the 28BYJ-48 stepper (vertical shaft) and
 *    4 diagonal struts towards the enclosure. There is no disc under the tray,
 *    so objects fall freely into the containers.
 *  - YOKE (rotation): a U-shaped bar fixed to the 28BYJ-48 shaft. It only turns
 *    between 0 and 90 degrees (two microswitches provide homing at those two
 *    positions) and carries the SG90 servo and the tilt axis.
 *  - BEAM + TRAY (tilt): the horizontal beam is the tilt axis. One end is a
 *    simple pivot, the other is coupled to the SG90 horn (direct drive,
 *    90 degrees of servo travel = +/-45 degrees of tilt).
 *  - The tray is concave (parabolic) and has 2 "bites" on the sides where the
 *    refuse falls (perpendicular to the tilt axis). They work as a funnel
 *    towards the container. The ends of the tilt axis keep a solid rim.
 *  - 4 containers at 0/90/180/270 degrees around the axis (translucent ghosts).
 *  - COUNTERWEIGHT: a holder under the beam with a sliding steel bar.
 *  - "L" or "arch" SUPPORT (cam_support) for the overhead Logitech C170 camera,
 *    bolted to a diagonal strut of the base. Its front wall (user side) carries
 *    3 x 8 mm LEDs, below them the SSD1306 OLED (not in the kit) with an acrylic
 *    window and, under a lower shelf, the IR light angle sensor tilted towards
 *    the tray (ir_aim). The OLED shows the gestures to perform and the IR sensor
 *    can also detect a hand approaching to drop refuse.
 *
 * Positioning logic (the +/-45 degree tilt reaches 2 containers for each yaw):
 *    yaw 0   -> tilt +/-45 reaches the containers at 90 and 270 degrees (+/-Y)
 *    yaw 90  -> tilt +/-45 reaches the containers at 0 and 180 degrees  (+/-X)
 *    If the target container is already "in front" of the current position the
 *    tray only tilts. Otherwise it rotates 90 degrees, stops and then tilts.
 *
 * Convention: positive yaw = counterclockwise seen from above.
 *             positive tilt = the +Y side goes up (refuse falls towards -Y).
 *
 * Units: mm.
 * Usage: change `part` to export single parts to STL.
 *        Set `animate = true` and use View > Animate (FPS 20, Steps 400) to run
 *        the sorting simulation of the four refuse types.
 *
 * Dimensions of the commercial parts (28BYJ-48, SG90, Logitech C170, SSD1306)
 * are typical values: check them against your own parts.
 */

// ------------------------------------------------------------------ PARAMETERS
part        = "assembly";   // "assembly","tray","beam","yoke","base","counterweight","camera_support"
yaw         = 0;            // [0:90]    tray rotation (28BYJ-48)
tilt        = 0;            // [-45:45]  tray tilt (SG90)
animate     = false;        // true: refuse deposition simulation (View > Animate)
show_bins   = true;         // ghost containers
show_hw     = true;         // motor, servo, horn, microswitches, sensors

$fn = 90;

// ------------------------------------------------ REFUSE DEPOSITION SIMULATION
// Container i sits at angle 90*i: 0=+X Packaging, 1=+Y Paper, 2=-X Organic, 3=-Y General waste
bin_names = ["Packaging","Paper","Organic","General waste"];
seq = [1,3,0,2];   // demo order: Paper, General waste (no rotation), Packaging (rotates 90), Organic (no rotation)

function yaw_for(i)  = (i==0 || i==2) ? 90 : 0;     // containers on X -> yaw 90
function tilt_for(i) = (i==0 || i==3) ? 45 : -45;   // tilt + : refuse falls towards the local -Y side
function clamp01(x)  = min(max(x,0),1);
function ease(x)     = let(c = clamp01(x)) c*c*(3-2*c);

// Timeline: $t in [0,1) -> 4 refuse items, each with its phases (u in [0,1)):
//  0.00-0.12 the item drops onto the tray        0.12-0.25 computer vision (identifies it)
//  0.25-0.40 rotation of 90 deg if needed (stop) 0.40-0.56 tilt to +/-45 deg
//  0.52-0.68 the item slides through the bite    0.68-0.86 falls into the container
//  0.72-0.88 the tray returns to 0 deg           0.90-1.00 (last item) yaw returns to 0 (rest)
sim_k   = min(floor($t*4), 3);
sim_u   = $t*4 - sim_k;
sim_bin = seq[sim_k];
sim_prev_yaw = sim_k==0 ? 0 : yaw_for(seq[sim_k-1]);
sim_yaw_raw  = sim_prev_yaw + (yaw_for(sim_bin)-sim_prev_yaw)*ease((sim_u-0.25)/0.15);
sim_yaw  = sim_k==3 ? sim_yaw_raw*(1-ease((sim_u-0.90)/0.10)) : sim_yaw_raw;
sim_tilt = tilt_for(sim_bin)*(ease((sim_u-0.40)/0.16) - ease((sim_u-0.72)/0.16));

yaw_e  = animate ? sim_yaw  : yaw;
tilt_e = animate ? sim_tilt : tilt;

// Refuse kinematics
orad   = 8;                                  // approximate size of each refuse item
function zloc(d) = beam_h/2 + hub_h + zs(d) + tray_t + orad;
function rx(p,a) = [p[0], p[1]*cos(a)-p[2]*sin(a), p[1]*sin(a)+p[2]*cos(a)];
function rz(p,a) = [p[0]*cos(a)-p[1]*sin(a), p[0]*sin(a)+p[1]*cos(a), p[2]];
function world_pos(yw,tl,lp) = rz(rx(lp,tl)+[0,0,Zt], yw);
function rest_pos(i) = [96*cos(90*i), 96*sin(90*i), -140+orad];

function item_pos(j) =
    j > sim_k ? undef :
    j < sim_k ? rest_pos(seq[j]) :
    (sim_u < 0.12) ? world_pos(yaw_e, tilt_e, [0,0, zloc(0) + 100*(1-pow(sim_u/0.12,2))]) :
    (sim_u < 0.52) ? world_pos(yaw_e, tilt_e, [0,0, zloc(0)]) :
    (sim_u < 0.68) ?
        (let(s = (sim_u-0.52)/0.16,
             d = d_exit*s*s,
             sg = tilt_for(sim_bin) > 0 ? -1 : 1)
            world_pos(yaw_e, tilt_e, [0, sg*d, zloc(d)])) :
        (let(sg = tilt_for(sim_bin) > 0 ? -1 : 1,
             p0 = world_pos(yaw_for(sim_bin), tilt_for(sim_bin), [0, sg*d_exit, zloc(d_exit)]),
             pr = rest_pos(sim_bin),
             s  = clamp01((sim_u-0.68)/0.18))
            [p0[0]+(pr[0]-p0[0])*s, p0[1]+(pr[1]-p0[1])*s, p0[2]+(pr[2]-p0[2])*s*s]);

module item_shape(i) {
    if      (i==0) color("gold")      cylinder(d=14, h=16, center=true);              // can (packaging)
    else if (i==1) color("royalblue") rotate([25,35,10]) cube(13, center=true);       // crumpled paper
    else if (i==2) color("firebrick") sphere(r=orad);                                 // fruit (organic)
    else           color("gray")      sphere(r=orad, $fn=6);                          // general waste
}

module items() {
    for (j=[0:3]) {
        p = item_pos(j);
        if (!is_undef(p)) translate(p) item_shape(seq[j]);
    }
}

// Tray
tray_d     = 150;
R          = tray_d/2;
tray_t     = 3;      // thickness
dish_depth = 14;     // depth of the concavity (parabola)
bite_r     = 22;     // radius of each bite
bite_off   = 4;      // offset of the bite centre beyond the rim
hub_d      = 46;     // base that fixes the tray to the beam
hub_h      = 4;

// Tilt axis
Zt     = 60;         // height of the tilt axis above the base plate
beam_w = 14;
beam_h = 12;

// Yoke
AX    = 83;          // inner face of the arms (|x|)
arm_t = 4;           // arm thickness (= thickness for the SG90 ears)

// Microswitches (rotation homing). Angle (on the base) where each one is pressed.
// Cam mounted at 90 deg at yaw=0 -> switch A at 90 deg (yaw 0), switch B at 180 deg (yaw +90)
sw_angles = [90, 180];

// Radius where the tray opens through the bite (the refuse leaves the tray here).
// Must come after R, bite_r and bite_off (OpenSCAD evaluates variables in order).
d_exit = R + bite_off - bite_r - 1;

// ------------------------------------------------------------------ COUNTERWEIGHT
// Printed holder under the beam + 15x15 mm steel bar (it slides through a
// through-cavity and is locked with an M3 grub screw). It balances the tray so
// the SG90 hardly works at rest.
cw_len     = 72;     // holder length (X axis)
cw_out     = 24;     // outer section (Y and Z)
cw_cav     = 16;     // through cavity
cw_bar_len = 24;     // steel bar length (adjust with a scale; must be < 56)
rho_pla    = 1.24e-3;   // g/mm3 (approx. PETG/PLA)
rho_steel  = 7.85e-3;   // g/mm3

// Rough estimate of the moments about the axis (g*mm)
m_plate  = (PI*R*R - 2*0.45*PI*bite_r*bite_r)*tray_t*rho_pla;
z_plate  = beam_h/2 + hub_h + dish_depth/2 + tray_t/2;
m_hub    = PI*pow(hub_d/2,2)*(hub_h+tray_t)*rho_pla;
z_hub    = beam_h/2 + (hub_h+tray_t)/2;
mom_tray = m_plate*z_plate + m_hub*z_hub;
m_holder = (cw_len*cw_out*cw_out - cw_len*cw_cav*cw_cav)*rho_pla;
z_cw     = beam_h/2 + cw_out/2;
mom_cw   = (m_holder + 15*15*cw_bar_len*rho_steel)*z_cw;
bar_ideal = max(0, (mom_tray - m_holder*z_cw)/(z_cw*15*15*rho_steel));

// ------------------------------------------- OVERHEAD CAMERA + IR + LED SUPPORT
// 20x20 profile (2020 aluminium or similar) on a diagonal strut (free of containers).
//   "L"    : one post on the cam_ang diagonal and a horizontal arm past the axis.
//   "arch" : arch with two posts on opposite diagonals (stiffer).
// In the local frame of the support: +x points from the axis to the post; the user
// stands on the -x side (that is why the LEDs face -x).
cam_support  = "L";            // "L" or "arch"
show_cam     = true;
cam_ang      = 45;             // 45, 135, 225 or 315 (where there is a strut)
cam_post_r   = 150;            // radius of the post on the strut
cam_dims     = [60.5, 70.3, 71]; // Logitech C170 (X, Y, height) from published data; check with your camera
cam_fov_diag = 58;             // C170: 58 deg diagonal FOV, fixed focus
cam_view_d   = 120;            // diameter of the tray area the camera must see
cage_wall    = 3;
cage_clear   = 0.6;
plate_t      = 4;
ir_dims      = [68.58,20.32,1.57]; // EVAL-CN0569-PMDZ (ADI wiki): length (Y axis), width (X axis), thickness
ir_pd_pitch  = 25.4;           // spacing of the two ADPD2140 photodiode sensors
ir_sp        = 5;              // height of the supports under the shelf
ir_hood_h    = 8;              // short hood (must not cut the angular field of view)
ir_wall      = 2.5;            // hood wall
ir_clear     = 0.5;            // clearance around the board
led_pitch    = 16;

// Lens height: a 4:3 camera has its short side = 0.6 * diagonal
cam_vfov   = 2*atan(0.6*tan(cam_fov_diag/2));
cam_dist   = (cam_view_d/2)/tan(cam_vfov/2);                 // lens -> tray plane
cam_lens_z = Zt + beam_h/2 + hub_h + tray_t + cam_dist;
plate_z    = cam_lens_z + 2*cage_wall + cam_dims[2] + 1 - cage_wall;  // underside of the head plate
arm_top    = plate_z + plate_t + 20;                         // total height of the post

// Front wall (user side): LEDs on top, OLED below and, on its underside, the IR sensor
panel_h    = 54;               // height of the wall hanging from the plate
panel_w    = 84;               // width of the wall (fits the 68.6 mm IR board)
led_z      = -13;              // height of the LED row relative to the underside of the plate
oled_pcb   = [27.5,27.5,1.6];  // SSD1306 0.96" module (approx.; check your module)
oled_win   = [24,13];          // display window
oled_hole  = [23,23.5];        // spacing of the M2 holes
oled_z     = -35;              // OLED centre relative to the underside of the plate
ir_head_x  = -57;              // x position of the IR sensor (under the shelf)
ir_aim     = atan(abs(ir_head_x)/(plate_z - panel_h - (Zt+beam_h/2+hub_h+tray_t)));  // tilt towards the tray centre

// ---------------------------------------------------------- COMPONENTS (mock-ups)
// 28BYJ-48: origin on the motor shaft, top face of the body at z = -3
module stepper28byj() {
    color("silver") translate([0,-8,-22]) cylinder(d=28, h=19);      // body (shaft offset 8 mm)
    color("silver") difference() {                                     // 35 mm mounting ears
        translate([-21,-8-3.5,-4]) cube([42,7,1]);
        for (x=[-17.5,17.5]) translate([x,-8,-5]) cylinder(d=4.2,h=3,$fn=24);
    }
    color("gold") translate([0,0,-3]) cylinder(d=9, h=1.5);           // collar
    color("gold") translate([0,0,-1.5]) intersection() {              // shaft with 2 flats
        cylinder(d=5, h=10);
        translate([-3,-1.5,0]) cube([6,3,10]);
    }
}

// SG90: origin on the output shaft, z=0 top face of the body, body towards -z
module sg90() {
    color("royalblue") {
        translate([-6,-6.1,-22.7]) cube([22.5,12.2,22.7]);            // body
        translate([-11,-6.1,-6.8]) difference() {                      // ears
            cube([32.5,12.2,2.5]);
            for (x=[2.5,30]) translate([x,6.1,-1]) cylinder(d=2.2,h=5,$fn=20);
        }
        cylinder(d=11.8, h=4);                                          // gear pedestal
    }
    color("white") cylinder(d=4.8, h=7.7);                              // splined shaft
}

module microswitch() {   // subminiature SPDT approximation (check the datasheet)
    color("red") cube([10.2,20,6.4]);
}

module shaft_socket(h) { // double-D socket for the 28BYJ-48 shaft (diameter 5, flats 3 mm)
    intersection() {
        cylinder(d=5.3, h=h);
        translate([-5,-1.65,0]) cube([10,3.3,h]);
    }
}

// ---------------------------------------------------------------------- BASE
module base() {
    difference() {
        union() {
            translate([-40,-40,-3]) cube([80,80,3]);                   // motor plate
            for (a=[45:90:315]) rotate(a) translate([30,-5,-6]) cube([140,10,6]);  // struts
        }
        translate([0,0,-4]) cylinder(d=10, h=6);                       // hole for the collar
        for (x=[-17.5,17.5]) translate([x,-8,-4]) cylinder(d=4.2,h=6,$fn=24);   // M4 motor screws
    }
}

// ---------------------------------------------------------------------- YOKE
module arm(s) {
    x0 = s>0 ? AX : -AX-arm_t;
    difference() {
        hull() {
            translate([x0,-11,10]) cube([arm_t,22,8]);
            translate([x0,0,Zt]) rotate([0,90,0]) cylinder(d=36, h=arm_t);
        }
        if (s>0) {   // SG90 window + screws
            translate([AX-1,-6.5,Zt-17]) cube([arm_t+2,13,23.5]);
            for (z=[Zt+8.5, Zt-19]) translate([AX-1,0,z]) rotate([0,90,0]) cylinder(d=2.2,h=arm_t+2,$fn=20);
        } else {     // pivot bushing hole
            translate([x0-1,0,Zt]) rotate([0,90,0]) cylinder(d=6.4,h=arm_t+2);
        }
    }
}

module yoke() {
    difference() {
        union() {
            translate([0,0,1]) cylinder(d=32, h=17);                           // hub
            translate([-(AX+arm_t),-11,10]) cube([2*(AX+arm_t),22,8]);         // bottom bar
            arm(1); arm(-1);
            rotate(90) translate([14,-3,2]) cube([15.5,6,5]);                  // cam for the microswitches
        }
        translate([0,0,0.99]) shaft_socket(7.8);                               // 28BYJ-48 shaft
        translate([0,0,5]) rotate([0,90,0]) cylinder(d=2.9, h=20, $fn=24);     // M3 grub screw
    }
}

// -------------------------------------------------------- BEAM (tilt axis)
// Local frame: origin on the tilt axis
module beam() {
    discx = AX-12.8;   // horn coupling disc: x = 70.2 .. 78.2
    difference() {
        union() {
            translate([-(AX-3),-beam_w/2,-beam_h/2]) cube([(AX-3)+discx+0.1, beam_w, beam_h]);
            translate([-(AX+3.5),0,0]) rotate([0,90,0]) cylinder(d=6, h=7);   // pivot
            translate([discx,0,0]) rotate([0,90,0]) cylinder(d=24, h=8);      // horn disc
        }
        translate([AX-4.8-4.5,-3.7,-11]) cube([4.6,7.4,22]);                   // horn pocket
        translate([discx-1,0,0]) rotate([0,90,0]) cylinder(d=3.2,h=10,$fn=24); // M2/M3 screw
        for (x=[-15,15]) translate([x,0,-beam_h/2-1]) cylinder(d=2.6,h=beam_h+2,$fn=20); // tray fixing
        for (x=[-30,30]) translate([x,0,-beam_h/2-1]) cylinder(d=2.6,h=9,$fn=20);          // counterweight fixing (blind)
    }
}

module horn() {   // single-arm horn inside its pocket (visual)
    color("white") translate([AX-4.8-4.0,0,0]) rotate([0,90,0]) cylinder(d=7, h=3.8);
    color("white") translate([AX-4.8-3.6,-3.4,-10]) cube([3.4,6.8,20]);
}

// ------------------------------------------------------------------- TRAY
function zs(r) = dish_depth*pow(r/R,2);

module dish() {
    N = 40;
    bottom = [for (i=[0:N]) let(r=R*i/N) [r, zs(r)]];
    top    = [for (i=[N:-1:0]) let(r=R*i/N) [r, zs(r)+tray_t]];
    rotate_extrude() polygon(concat(bottom, top));
}

// Local frame: origin on the tilt axis
module tray() {
    z0 = beam_h/2;
    difference() {
        union() {
            translate([0,0,z0+hub_h]) dish();
            translate([0,0,z0]) cylinder(d=hub_d, h=hub_h+tray_t+0.01);
        }
        for (a=[90,270]) rotate(a) translate([R+bite_off,0,0]) cylinder(r=bite_r, h=80);     // 2 bites (fall sides, perpendicular to the tilt axis)
        for (x=[-15,15]) translate([x,0,z0-1]) cylinder(d=3.4,h=hub_h+tray_t+2,$fn=20);
    }
}

// ----------------------------------------------------------------- COUNTERWEIGHT
// Local frame: origin on the tilt axis (it hangs under the beam).
// Maximum swing radius is about 32 mm: it leaves about 10 mm of clearance over the yoke (42 mm below the axis).
module counterweight() {
    z_top = -beam_h/2;
    difference() {
        translate([-cw_len/2, -cw_out/2, z_top-cw_out]) cube([cw_len, cw_out, cw_out]);
        translate([-cw_len/2-1, -cw_cav/2, z_top-cw_out/2-cw_cav/2]) cube([cw_len+2, cw_cav, cw_cav]);  // cavity
        for (x=[-30,30]) translate([x,0,z_top-cw_out-1]) cylinder(d=3.4, h=cw_out+2, $fn=20);            // M3 to the beam
        for (x=[-15,15]) translate([x,0,z_top-6]) cylinder(d=4, h=8, $fn=20);                           // clearance for the tray screws
        translate([0,0,z_top-cw_out-1]) cylinder(d=2.6, h=6, $fn=20);                                   // M3 grub screw
    }
}

module counterweight_bar() {   // 15x15 steel bar (visual)
    translate([-cw_bar_len/2, -7.5, -beam_h/2-cw_out/2-7.5]) cube([cw_bar_len, 15, 15]);
}

// IR sensor head: the mounting face is at z=0 and the sensor looks towards -z.
// The board is placed with its long axis along Y (left-right of the user).
ir_vis_x = ir_dims[1] + 2*ir_clear + 2*ir_wall;   // outer dimensions of the hood
ir_vis_y = ir_dims[0] + 2*ir_clear + 2*ir_wall;

module ir_head() {
    h = ir_sp + ir_dims[2] + ir_hood_h;
    color("orange") translate([0,0,-h]) difference() {                  // rectangular hood
        translate([-ir_vis_x/2,-ir_vis_y/2,0]) cube([ir_vis_x,ir_vis_y,h]);
        translate([-ir_dims[1]/2-ir_clear,-ir_dims[0]/2-ir_clear,-1])
            cube([ir_dims[1]+2*ir_clear, ir_dims[0]+2*ir_clear, h+2]);
        // Notch for the Pmod connector cable (+Y end)
        translate([-10, ir_dims[0]/2, h-ir_sp-ir_dims[2]-4]) cube([20, ir_wall+4, 8]);
    }
    // End supports (printed clips): the ADI page gives no mounting holes
    color("orange") for (sy=[-1,1]) translate([-4,sy*(ir_dims[0]/2-3)-3,-ir_sp]) cube([8,6,ir_sp]);
    if (show_hw) {
        // PCB 68.58 x 20.32 x 1.57
        color("green") translate([-ir_dims[1]/2,-ir_dims[0]/2,-ir_sp-ir_dims[2]]) cube([ir_dims[1],ir_dims[0],ir_dims[2]]);
        // Two ADPD2140 sensors 25.4 mm apart with their baffles (approximate position; see the assembly drawing)
        for (sy=[-1,1]) translate([0,sy*ir_pd_pitch/2,-ir_sp-ir_dims[2]]) {
            color("black") translate([-3,-3,-1.2]) cube([6,6,1.2]);
            color("dimgray") translate([0,0,-4]) difference() {
                translate([-5,-5,0]) cube([10,10,4]);
                translate([-3.2,-3.2,-1]) cube([6.4,6.4,6]);
            }
        }
        // Infrared LED (approximate position, in the centre)
        color("darkred") translate([0,0,-ir_sp-ir_dims[2]]) translate([0,0,-1.5]) cylinder(d=3,h=1.5,$fn=20);
        // I2C Pmod 1x6 connector (15.24 mm long) at the +Y end (approximate position)
        color("gold") translate([-(15.24+2.6)/2,ir_dims[0]/2-3,-ir_sp-ir_dims[2]-2.5]) cube([15.24+2.6,2.6,2.5]);
    }
}

// Front wall with 3 x 8 mm LEDs, an OLED with an acrylic window and a lower shelf for the IR sensor
module front_panel() {
    xo = -75;   // outer face, towards the user
    color("orange") translate([0,0,plate_z]) difference() {
        union() {
            translate([xo,-panel_w/2,-panel_h]) cube([4,panel_w,panel_h]);   // wall
            translate([xo,-panel_w/2,-panel_h]) cube([36,panel_w,5]);        // IR sensor shelf
        }
        for (i=[-1:1]) translate([xo-1,i*led_pitch,led_z]) rotate([0,90,0]) cylinder(d=8.2,h=6,$fn=32);   // LEDs
        translate([xo-1,-oled_win[0]/2,oled_z-oled_win[1]/2]) cube([6,oled_win[0],oled_win[1]]);          // OLED window
        translate([xo-1,-15,oled_z-9.5]) cube([2.5,30,19]);                                               // 1.5 mm acrylic recess
        for (dx=[-1,1], dy=[-1,1]) translate([xo-1,dx*oled_hole[0]/2,oled_z+dy*oled_hole[1]/2])
            rotate([0,90,0]) cylinder(d=2.2,h=6,$fn=20);                                                  // OLED M2 holes
    }
}

// ------------------------------------------------ OVERHEAD CAMERA + IR + LED SUPPORT
module camera_support() {
    ia  = (cam_support == "arch");
    cx  = cam_dims[0] + 2*cage_clear;
    cy  = cam_dims[1] + 2*cage_clear;
    cage_h = plate_z - (cam_lens_z - cage_wall);
    rotate(cam_ang) {
        // Posts with a foot bolted to the diagonal strut (4 x M4)
        for (s = ia ? [1,-1] : [1]) translate([s*cam_post_r,0,0]) {
            color("silver") translate([-10,-10,5]) cube([20,20,arm_top-5]);
            color("dimgray") translate([-25,-25,0]) difference() {
                cube([50,50,5]);
                for (dx=[-18,18], dy=[-18,18]) translate([25+dx,25+dy,-1]) cylinder(d=4.2,h=7,$fn=20);
            }
        }
        // Arm / arch
        color("silver") translate([ia ? -(cam_post_r+10) : -70, -10, plate_z+plate_t])
            cube([ia ? 2*(cam_post_r+10) : cam_post_r+10+70, 20, 20]);
        // Head plate (fixed to the profile with 4 x M4)
        color("orange") translate([-75,-45,plate_z]) cube([120,90,plate_t]);
        // Camera cage: the lens looks down, centred on the axis
        color("orange") translate([0,0,cam_lens_z-cage_wall]) difference() {
            translate([-cx/2-cage_wall, -cy/2-cage_wall, 0]) cube([cx+2*cage_wall, cy+2*cage_wall, cage_h]);
            translate([-cx/2,-cy/2,cage_wall]) cube([cx,cy,cage_h]);
            translate([0,0,-1]) cylinder(d=30, h=cage_wall+2);                         // lens window
            translate([cx/2-1,-7,cage_h-30]) cube([cage_wall+2,14,20]);                // USB cable exit
        }
        // Front wall: LEDs on top, OLED below and a lower shelf for the IR sensor
        front_panel();
        // IR sensor under the shelf, tilted to aim at the tray centre
        translate([ir_head_x,0,plate_z-panel_h]) {
            color("orange") hull() {                                  // tilt wedge
                translate([-ir_vis_x/2,-ir_vis_y/2,-0.01]) cube([ir_vis_x,ir_vis_y,0.01]);
                rotate([0,-ir_aim,0]) translate([-ir_vis_x/2,-ir_vis_y/2,-0.01]) cube([ir_vis_x,ir_vis_y,0.01]);
            }
            rotate([0,-ir_aim,0]) ir_head();
        }
        if (show_hw) {
            cols = ["limegreen","orange","red"];
            for (i=[0:2]) translate([-71,(i-1)*led_pitch,plate_z+led_z]) {
                color(cols[i]) rotate([0,-90,0]) cylinder(d=8,h=8,$fn=32);
                color("dimgray") rotate([0,90,0]) cylinder(d=10,h=12,$fn=32);
            }
            // SSD1306 OLED: glass against the inner face of the wall, PCB behind
            color("black") translate([-71,-13.5,plate_z+oled_z-7.5]) cube([1.4,27,15]);
            color("navy")  translate([-71+1.4,-oled_pcb[0]/2,plate_z+oled_z-oled_pcb[1]/2]) cube([oled_pcb[2],oled_pcb[0],oled_pcb[1]]);
            // Camera (bounding volume and lens)
            color("black",0.55) translate([-cam_dims[0]/2,-cam_dims[1]/2,cam_lens_z]) cube(cam_dims);
            color("navy") translate([0,0,cam_lens_z-0.5]) cylinder(d=14,h=2);
        }
    }
}

// ----------------------------------------------------------------- CONTAINERS
module bins() {
    cols = ["gold","royalblue","saddlebrown","gray"];   // Packaging, Paper, Organic, General waste
    for (i=[0:3]) rotate(i*90) color(cols[i], 0.25) translate([46,-40,-140]) cube([100,80,130]);
}

// ------------------------------------------------------------------- ASSEMBLY
module assembly() {
    color("dimgray") base();
    if (show_hw) {
        stepper28byj();
        color("black") translate([0,0,0]) cylinder(d=26,h=1);                    // sliding washer
        for (a=sw_angles) rotate(a) translate([28.5,-10,0]) microswitch();
    }
    rotate([0,0,yaw_e]) {
        color("orange") yoke();
        if (show_hw) translate([AX-0.3,0,Zt]) rotate([0,-90,0]) rotate([0,0,180]) sg90();
        translate([0,0,Zt]) rotate([tilt_e,0,0]) {
            color("steelblue") beam();
            color("seagreen") tray();
            color("slategray") counterweight();
            if (show_hw) { horn(); color("dimgray") counterweight_bar(); }
        }
    }
    if (show_cam) camera_support();
    if (show_bins) bins();
    if (animate) items();
}

// -------------------------------------------------------------------- OUTPUT
if      (part=="assembly")        assembly();
else if (part=="tray")            tray();
else if (part=="beam")            beam();
else if (part=="yoke")            yoke();
else if (part=="base")            base();
else if (part=="counterweight")   counterweight();
else if (part=="camera_support")  camera_support();

echo(str("Tray diameter ", tray_d, " mm | tilt axis at ", Zt, " mm | yaw=", yaw_e, " tilt=", tilt_e));
echo(str("Balance: tray moment ~ ", round(mom_tray), " g*mm | counterweight ~ ", round(mom_cw),
         " g*mm | balancing bar ~ ", round(bar_ideal*10)/10, " mm (about 10-20 % more makes the tray rest at 0 deg)"));
echo(str("IR sensor: tilt towards the tray ~ ", round(ir_aim*10)/10, " deg"));
echo(str("Camera: lens at ", round(cam_lens_z), " mm (", round(cam_dist), " mm above the tray, sees a diameter of ", cam_view_d,
         " mm) | total support height ~ ", round(arm_top), " mm | type: ", cam_support));
if (animate) echo(str("Refuse: ", bin_names[sim_bin], " -> container at ", 90*sim_bin, " deg | yaw=", round(yaw_e), " tilt=", round(tilt_e)));
