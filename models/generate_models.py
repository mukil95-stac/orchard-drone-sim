from pathlib import Path
import math
import cv2

ROOT=Path(__file__).resolve().parents[1]

def drone_visual_parts():
    """Huge 18-inch deadcat frame with a protected forward onboard camera."""
    parts=['''<geom name="lower_carbon_plate" type="box" pos="0 0 -.020" size=".13 .15 .014" rgba=".035 .045 .055 1" contype="0" conaffinity="0"/>
<geom name="upper_carbon_plate" type="box" pos="0 -.014 .027" size=".105 .125 .012" rgba=".075 .085 .095 1" contype="0" conaffinity="0"/>
<geom name="front_left_arm" type="capsule" fromto="-.040 .040 .005 -.400 .400 .005" size=".024" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="front_right_arm" type="capsule" fromto=".040 .040 .005 .400 .400 .005" size=".024" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="rear_left_arm" type="capsule" fromto="-.040 -.040 .005 -.315 -.315 .005" size=".024" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="rear_right_arm" type="capsule" fromto=".040 -.040 .005 .315 -.315 .005" size=".024" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="front_camera_mount" type="box" pos="0 .260 .020" size=".065 .042 .038" rgba=".12 .13 .14 1" contype="0" conaffinity="0"/>
<geom name="camera_lens_rim" type="capsule" fromto="0 .300 .020 0 .335 .020" size=".022" rgba=".025 .03 .04 1" contype="0" conaffinity="0"/>
<geom name="camera_lens_glass" type="sphere" pos="0 .345 .020" size=".014" rgba=".12 .32 .48 1" contype="0" conaffinity="0"/>
<geom name="camera_guard_left" type="capsule" fromto="-.060 .210 .010 -.057 .290 .010" size=".008" rgba=".22 .23 .24 1" contype="0" conaffinity="0"/>
<geom name="camera_guard_right" type="capsule" fromto=".060 .210 .010 .057 .290 .010" size=".008" rgba=".22 .23 .24 1" contype="0" conaffinity="0"/>
<geom name="camera_frame_rail_left" type="capsule" fromto="-.075 .115 .005 -.048 .245 .012" size=".012" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="camera_frame_rail_right" type="capsule" fromto=".075 .115 .005 .048 .245 .012" size=".012" rgba=".10 .11 .12 1" contype="0" conaffinity="0"/>
<geom name="camera_nose_bridge" type="box" pos="0 .220 .012" size=".055 .018 .012" rgba=".075 .085 .095 1" contype="0" conaffinity="0"/>
<geom name="flight_controller_stack" type="cylinder" pos="0 -.014 .050" size=".038 .015" rgba=".10 .16 .18 1" contype="0" conaffinity="0"/>
<geom name="top_lidar_mount" type="cylinder" pos="0 0 .095" size=".045 .012" rgba=".16 .18 .20 1" contype="0" conaffinity="0"/>
<geom name="top_lidar_sensor" type="cylinder" pos="0 0 .122" size=".032 .018" rgba=".05 .32 .38 1" contype="0" conaffinity="0"/>
<geom name="landing_leg_front_left" type="capsule" fromto="-.105 .115 -.025 -.155 .165 -.235" size=".012" rgba=".16 .17 .18 1" contype="1" conaffinity="1"/>
<geom name="landing_leg_front_right" type="capsule" fromto=".105 .115 -.025 .155 .165 -.235" size=".012" rgba=".16 .17 .18 1" contype="1" conaffinity="1"/>
<geom name="landing_leg_rear_left" type="capsule" fromto="-.105 -.115 -.025 -.155 -.165 -.235" size=".012" rgba=".16 .17 .18 1" contype="1" conaffinity="1"/>
<geom name="landing_leg_rear_right" type="capsule" fromto=".105 -.115 -.025 .155 -.165 -.235" size=".012" rgba=".16 .17 .18 1" contype="1" conaffinity="1"/>
''']
    rotors=[(-.400,.400),(-.315,-.315),(.315,-.315),(.400,.400)]
    for n,(x,y) in enumerate(rotors,1):
        # Balance propeller visuals around the origin despite the deadcat frame's unequal front/rear arm lengths.
        prop_mass=.0007875 if n in (1,4) else .001
        parts.append(f'<geom name="motor_can_{n}" type="cylinder" pos="{x} {y} .016" size=".028 .018" rgba=".17 .18 .19 1" contype="0" conaffinity="0"/><geom name="motor_bell_{n}" type="cylinder" pos="{x} {y} .038" size=".022 .008" rgba=".42 .44 .46 1" contype="0" conaffinity="0"/><body name="propeller_{n}" pos="{x} {y} .048"><joint name="propeller_spin_{n}" type="hinge" axis="0 0 1" damping="0" armature=".000001"/><inertial pos="0 0 0" mass="{prop_mass}" diaginertia="{prop_mass*.001} {prop_mass*.017} {prop_mass*.017}"/><geom name="prop_blade_{n}" type="capsule" fromto="-.225 0 0 .225 0 0" size=".006" rgba=".08 .09 .10 .85" contype="0" conaffinity="0"/><geom name="propeller_tip_{n}" type="sphere" pos=".205 0 .002" size=".010" rgba="1 .28 .025 .95" contype="0" conaffinity="0"/></body>')
    return ''.join(parts)

def generate():
    assets=''.join(f'<texture name="m{i}" type="2d" file="../assets/aruco_{i:02d}.png"/><material name="mm{i}" texture="m{i}" texuniform="true"/>' for i in range(1,10))
    world=['''<mujoco model="orchard_whoop"><compiler angle="radian" inertiafromgeom="false"/><option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/><visual><global offwidth="640" offheight="480"/><map znear=".01"/><headlight diffuse=".72 .72 .72" ambient=".48 .48 .52" specular=".12 .12 .12"/></visual><asset><texture name="day_sky" type="skybox" builtin="gradient" rgb1=".54 .76 .96" rgb2=".92 .96 1" width="512" height="512"/><material name="soil" rgba=".24 .38 .16 1"/><material name="pad" rgba=".18 .20 .20 1"/><material name="pad_marking" rgba=".92 .92 .84 1"/><material name="bark" rgba=".25 .12 .055 1"/><material name="bark_light" rgba=".36 .20 .10 1"/><material name="leaf" rgba=".12 .38 .12 1"/><material name="leaf_light" rgba=".20 .48 .15 1"/><material name="leaf_dark" rgba=".08 .29 .10 1"/><material name="leaf_lime" rgba=".48 .62 .10 1"/><material name="leaf_copper" rgba=".72 .25 .055 1"/><material name="apple" rgba=".88 .035 .018 1"/><material name="apple_gold" rgba=".98 .72 .08 1"/><material name="apple_green" rgba=".30 .72 .055 1"/><material name="apple_bad" rgba=".34 .22 .055 1"/><material name="apple_bruise" rgba=".20 .13 .035 1"/><material name="apple_blush" rgba=".82 .045 .025 1"/><material name="stake" rgba=".42 .30 .16 1"/>
'''+assets+'''</asset><worldbody><light pos="0 -8 20" dir="0 .3 -1" directional="true"/><light pos="-12 4 9" dir=".5 -.2 -1" directional="true"/><geom name="ground" type="plane" size="30 30 .1" material="soil"/><geom name="helipad" type="cylinder" pos="0 -13 .006" size="1.35 .006" material="pad" contype="0" conaffinity="0"/><geom name="helipad_H_left" type="box" pos="-.38 -13 .014" size=".07 .48 .008" material="pad_marking" contype="0" conaffinity="0"/><geom name="helipad_H_right" type="box" pos=".38 -13 .014" size=".07 .48 .008" material="pad_marking" contype="0" conaffinity="0"/><geom name="helipad_H_crossbar" type="box" pos="0 -13 .014" size=".38 .07 .008" material="pad_marking" contype="0" conaffinity="0"/>
''']
    # Enclose the orchard and launch pad with a low fence, leaving the flight area inside clear.
    fence_sides=[('north',(-12.,11.5),(12.,11.5)),('south',(-12.,-16.5),(12.,-16.5)),
                 ('west',(-12.,-16.5),(-12.,11.5)),('east',(12.,-16.5),(12.,11.5))]
    for side,(x0,y0),(x1,y1) in fence_sides:
        length=math.hypot(x1-x0,y1-y0); sections=math.ceil(length/2.5)
        for n in range(sections+1):
            f=n/sections; x=x0+(x1-x0)*f; y=y0+(y1-y0)*f
            world.append(f'<geom name="fence_{side}_post_{n:02d}" type="cylinder" pos="{x:.3f} {y:.3f} .675" size=".065 .675" material="stake"/>')
        for n in range(sections):
            fa=n/sections; fb=(n+1)/sections
            xa=x0+(x1-x0)*fa; ya=y0+(y1-y0)*fa
            xb=x0+(x1-x0)*fb; yb=y0+(y1-y0)*fb
            xm=(xa+xb)/2; ym=(ya+yb)/2; half=math.hypot(xb-xa,yb-ya)/2
            if side in ('north','south'):
                lower=f'<geom name="fence_{side}_rail_low_{n:02d}" type="box" pos="{xm:.3f} {ym:.3f} .46" size="{half:.3f} .045 .055" material="stake"/>'
                upper=f'<geom name="fence_{side}_rail_high_{n:02d}" type="box" pos="{xm:.3f} {ym:.3f} 1.08" size="{half:.3f} .045 .055" material="stake"/>'
            else:
                lower=f'<geom name="fence_{side}_rail_low_{n:02d}" type="box" pos="{xm:.3f} {ym:.3f} .46" size=".045 {half:.3f} .055" material="stake"/>'
                upper=f'<geom name="fence_{side}_rail_high_{n:02d}" type="box" pos="{xm:.3f} {ym:.3f} 1.08" size=".045 {half:.3f} .055" material="stake"/>'
            world.extend((lower,upper))
    for i in range(1,10):
        x=((i-1)%3-1)*8; y=(1-(i-1)//3)*8
        world.append(f'<body name="tree_{i:02d}" pos="{x} {y} 0"><geom name="trunk_{i:02d}" type="cylinder" pos="0 0 1.0" size=".19 1.0" material="bark"/><geom name="trunk_bark_ridge_{i:02d}_1" type="capsule" fromto="-.11 -.12 .12 -.08 -.10 1.76" size=".025" material="bark_light" contype="0" conaffinity="0"/><geom name="trunk_bark_ridge_{i:02d}_2" type="capsule" fromto=".12 -.08 .25 .10 -.07 1.55" size=".020" material="bark_light" contype="0" conaffinity="0"/><geom name="root_{i:02d}_1" type="capsule" fromto="0 0 .28 -.52 -.24 .06" size=".09" material="bark" contype="0" conaffinity="0"/><geom name="root_{i:02d}_2" type="capsule" fromto="0 0 .25 .50 -.18 .06" size=".08" material="bark" contype="0" conaffinity="0"/><geom name="root_{i:02d}_3" type="capsule" fromto="0 0 .25 .12 .48 .06" size=".075" material="bark" contype="0" conaffinity="0"/>')
        # Forked limbs give the canopy a visible tree structure beneath the foliage.
        branches=[((0,0,1.25),(-.92,-.58,2.45),.105),((0,0,1.45),(.88,-.48,2.62),.10),((0,0,1.55),(-.50,.82,2.72),.095),((0,0,1.62),(.58,.72,2.82),.09),((0,0,1.72),(0,.08,3.38),.075)]
        for b,(start,end,radius) in enumerate(branches,1):
            world.append(f'<geom name="branch_{i:02d}_{b}" type="capsule" fromto="{start[0]} {start[1]} {start[2]} {end[0]} {end[1]} {end[2]}" size="{radius}" material="bark" contype="0" conaffinity="0"/>')
        # Offset foliage clumps break up the single-ball silhouette of the old canopy.
        world.append(f'<geom name="canopy_{i:02d}" type="ellipsoid" pos="0 0 2.65" size="1.18 1.10 1.02" material="leaf" contype="0" conaffinity="0"/><geom name="canopy_light_{i:02d}" type="ellipsoid" pos="-.38 -.12 2.95" size=".78 .78 .66" material="leaf_light" contype="0" conaffinity="0"/><geom name="canopy_dark_{i:02d}" type="ellipsoid" pos=".42 .18 2.48" size=".82 .78 .70" material="leaf_dark" contype="0" conaffinity="0"/>')
        foliage=[(-.82,-.30,2.52,.48,.48,.43),(.78,-.35,2.68,.48,.44,.46),(-.38,.70,2.66,.50,.45,.47),(.47,.66,2.90,.47,.43,.43),(-.48,-.08,3.35,.53,.48,.44),(.42,-.12,3.30,.50,.45,.46),(.02,.18,3.58,.40,.38,.35)]
        for f,(fx,fy,fz,sx,sy,sz) in enumerate(foliage,1):
            mat='leaf_light' if f%3==0 else ('leaf_dark' if f%3==1 else 'leaf')
            world.append(f'<geom name="foliage_{i:02d}_{f}" type="ellipsoid" pos="{fx} {fy} {fz}" size="{sx} {sy} {sz}" material="{mat}" contype="0" conaffinity="0"/>')
        world.append(f'<geom name="foliage_lime_{i:02d}" type="ellipsoid" pos="-.62 -.72 2.76" size=".34 .28 .30" material="leaf_lime" contype="0" conaffinity="0"/><geom name="foliage_copper_{i:02d}" type="ellipsoid" pos=".58 -.68 2.88" size=".30 .25 .28" material="leaf_copper" contype="0" conaffinity="0"/>')
        if i==4:
            # A single visible fruit gives the first inspection tree a concrete scan target.
            world.append('<geom name="scan_apple_04" type="sphere" pos="-.32 -1.18 2.60" size=".16" material="apple" contype="0" conaffinity="0"/>')
        # Print the actual OpenCV dictionary code as lightweight square geoms on a matte board.
        board=.24; cell=board/6
        # Mount each south-facing marker on the tree's east side, clear of the canopy.
        # The inspection camera approaches from the south and can still see the board.
        marker_x=1.8; marker_y=-.665; marker_z=.95
        # Compact timber supports hold each board just above the tree-side path.
        world.append(f'<geom name="support_marker_{i:02d}" type="cylinder" pos="{marker_x} {marker_y} .315" size=".014 .315" material="stake"/><geom name="mount_marker_{i:02d}" type="box" pos="{marker_x} {marker_y} .645" size=".075 .04 .035" material="stake"/>')
        world.append(f'<geom name="marker_{i:02d}" type="box" pos="{marker_x} {marker_y} {marker_z}" size=".26 .004 .26" rgba="1 1 1 1" contype="0" conaffinity="0"/>')
        code=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),i,600,borderBits=1)
        for row in range(6):
            for col in range(6):
                if code[int((row+.5)*100),int((col+.5)*100)]<128:
                    xx=(col-2.5)*cell; zz=(2.5-row)*cell
                    world.append(f'<geom name="cell_{i:02d}_{row}_{col}" type="box" pos="{marker_x+xx} -.670 {marker_z+zz}" size="{cell/2} .003 {cell/2}" rgba=".005 .005 .005 1" contype="0" conaffinity="0"/>')
        world.append('</body>')
    rotors=[(-.400,.400),(-.315,-.315),(.315,-.315),(.400,.400)]
    world.append('''<body name="whoop" pos="0 -13 .25"><freejoint name="root"/><inertial pos="0 0 0" mass="2.0" diaginertia=".08 .08 .14"/><geom name="body" type="ellipsoid" size=".13 .15 .050" rgba=".08 .09 .11 1" contype="1" conaffinity="1"/>'''+drone_visual_parts()+'''<site name="imu"/><camera name="front_camera" pos="0 .260 .020" quat=".7071068 .7071068 0 0" fovy="75"/>
''')
    for n,(x,y) in enumerate(rotors,1):
        world.append(f'<site name="rotor_{n}" pos="{x} {y} 0" size=".012" rgba=".1 .5 .8 .7"/>')
    lidar_sites=[]
    lidar_sensors=[]
    for beam in range(16):
        angle=2*math.pi*beam/16
        # MuJoCo rangefinders measure along a site's local +Z axis. Rotate each
        # site so the top-mounted ring provides a horizontal 360-degree scan.
        half=angle/2
        # qz(angle) * qy(90deg): local +Z points horizontally at this angle.
        c=math.cos(math.pi/4); s=math.sin(math.pi/4)
        lidar_sites.append(f'<site name="lidar_{beam:02d}" pos="0 0 .145" quat="{math.cos(half)*c:.7f} {-math.sin(half)*s:.7f} {math.cos(half)*s:.7f} {math.sin(half)*c:.7f}" size=".006" rgba=".05 .8 .9 .7"/>')
        lidar_sensors.append(f'<rangefinder name="lidar_{beam:02d}" site="lidar_{beam:02d}" cutoff="30"/>')
    lidar_xml=''.join(lidar_sites)
    sensor_xml='<sensor>'+''.join(lidar_sensors)+'</sensor>'
    world.append(lidar_xml+'</body></worldbody>'+sensor_xml+'</mujoco>')
    (ROOT/'models'/'orchard.xml').write_text(''.join(world))
    parts=''.join(f'<site name="rotor_{n}" pos="{x} {y} 0" size=".018"/>' for n,(x,y) in enumerate(rotors,1))
    (ROOT/'models'/'drone.xml').write_text('''<mujoco model="huge_deadcat"><compiler angle="radian" inertiafromgeom="false"/><option timestep=".002" gravity="0 0 -9.81"/><worldbody><geom name="floor" type="plane" size="5 5 .1"/><body name="whoop" pos="0 0 1"><freejoint name="root"/><inertial pos="0 0 0" mass="2.0" diaginertia=".08 .08 .14"/><geom name="body" type="ellipsoid" size=".13 .15 .050"/>'''+drone_visual_parts()+'<site name="imu"/><camera name="front_camera" pos="0 .260 .020" quat=".7071068 .7071068 0 0" fovy="75"/>'+parts+lidar_xml+'</body></worldbody>'+sensor_xml+'</mujoco>')

if __name__=='__main__': generate()
