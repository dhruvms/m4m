#Copyright 2015 Yale University - Grablab
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import sys
import json
import urllib
import urllib2

output_directory = "../dat/ycb"

# You can either set this to "all" or a list of the objects that you'd like to
# download.
#objects_to_download = "all"
#objects_to_download = ["002_master_chef_can", "003_cracker_box"]
objects_to_download = [
    '002_master_chef_can',
    '003_cracker_box',
    '004_sugar_box',
    '005_tomato_soup_can',
    '006_mustard_bottle',
    '007_tuna_fish_can',
    '008_pudding_box',
    '009_gelatin_box',
    '010_potted_meat_can',
    '011_banana',
    '019_pitcher_base',
    '021_bleach_cleanser',
    '024_bowl',
    '025_mug',
    '035_power_drill',
    '036_wood_block'
]

# You can edit this list to only download certain kinds of files.
# 'berkeley_rgbd' contains all of the depth maps and images from the Carmines.
# 'berkeley_rgb_highres' contains all of the high-res images from the Canon cameras.
# 'berkeley_processed' contains all of the segmented point clouds and textured meshes.
# 'google_16k' contains google meshes with 16k vertices.
# 'google_64k' contains google meshes with 64k vertices.
# 'google_512k' contains google meshes with 512k vertices.
# See the website for more details.
#files_to_download = ["berkeley_rgbd", "berkeley_rgb_highres", "berkeley_processed", "google_16k", "google_64k", "google_512k"]
files_to_download = ["berkeley_processed", "google_16k"]

# Extract all files from the downloaded .tgz, and remove .tgz files.
# If false, will just download all .tgz files to output_directory
extract = True
urdf = True

base_url = "http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/"
objects_url = base_url + "objects.json"

object_masses = {'001_chips_can': 0.205, '002_master_chef_can': 0.414, '003_cracker_box': 0.411, '004_sugar_box': 0.514, '005_tomato_soup_can': 0.349, '006_mustard_bottle': 0.603, '007_tuna_fish_can': 0.171, '008_pudding_box': 0.187, '009_gelatin_box': 0.97, '010_potted_meat_can': 0.370, '011_banana': 0.066, '012_strawberry': 0.018, '013_apple': 0.068, '014_lemon': 0.029, '015_peach': 0.033, '016_pear': 0.049, '017_orange': 0.049, '018_plum': 0.025, '019_pitcher_base': 0.178, '021_bleach_cleanser': 1.131, '022_windex_bottle': 1.022, '023_wine_glass': 0.133, '024_bowl': 0.147, '025_mug': 0.118, '026_sponge': 0.0062, '027_skillet': 0.95, '028_skillet_lid': 0.652, '029_plate': 0.279, '030_fork': 0.034, '031_spoon': 0.03, '032_knife': 0.031, '033_spatula': 0.0515, '035_power_drill': 0.895, '036_wood_block': 0.729, '037_scissors': 0.082, '038_padlock': 0.304, '039_key': 0.0101, '040_large_marker': 0.0158, '041_small_marker': 0.0082, '042_adjustable_wrench': 0.252, '043_phillips_screwdriver': 0.097, '044_flat_screwdriver': 0.0984, '046_plastic_bolt': 0.0036, '047_plastic_nut': 0.001, '048_hammer': 0.665, '049_small_clamp': 0.0192, '050_medium_clamp': 0.059, '051_large_clamp': 0.125, '052_extra_large_clamp': 0.202, '053_mini_soccer_ball': 0.123, '054_softball': 0.191, '055_baseball': 0.148, '056_tennis_ball': 0.058, '057_racquetball': 0.041, '058_golf_ball': 0.046, '059_chain': 0.098, '061_foam_brick': 0.028, '062_dice': 0.0052, '063-a_marbles': 0.01, '063-b_marbles': 0.01, '063-c_marbles': 0.01, '063-d_marbles': 0.01, '063-e_marbles': 0.01, '063-f_marbles': 0.01, '065-a_cups': 0.013, '065-b_cups': 0.014, '065-c_cups': 0.017, '065-d_cups': 0.019, '065-e_cups': 0.021, '065-f_cups': 0.026, '065-g_cups': 0.028, '065-h_cups': 0.031, '065-i_cups': 0.035, '065-j_cups': 0.038, '070-a_colored_wood_blocks': 0.0108, '070-b_colored_wood_blocks': 0.0108, '071-a_nine_hole_peg_test': 1.435, '071-b_nine_hole_peg_test': 0.01, '071-c_nine_hole_peg_test': 0.01, '072-a_toy_airplane': 0.01, '072-b_toy_airplane': 0.01, '072-c_toy_airplane': 0.01, '072-d_toy_airplane': 0.01, '072-e_toy_airplane': 0.01, '072-f_toy_airplane': 0.01, '072-g_toy_airplane': 0.01, '072-h_toy_airplane': 0.01, '072-i_toy_airplane': 0.01, '072-j_toy_airplane': 0.01, '072-k_toy_airplane': 0.01, '073-a_lego_duplo': 0.01, '073-b_lego_duplo': 0.01, '073-c_lego_duplo': 0.01, '073-d_lego_duplo': 0.01, '073-e_lego_duplo': 0.01, '073-f_lego_duplo': 0.01, '073-g_lego_duplo': 0.01, '073-h_lego_duplo': 0.01, '073-i_lego_duplo': 0.01, '073-j_lego_duplo': 0.01, '073-k_lego_duplo': 0.01, '073-l_lego_duplo': 0.01, '073-m_lego_duplo': 0.01, '076_timer': 0.102, '077_rubiks_cube': 0.094}

if not os.path.exists(output_directory):
    os.makedirs(output_directory)

def fetch_objects(url):
    response = urllib2.urlopen(url)
    html = response.read()
    objects = json.loads(html)
    return objects["objects"]

def download_file(url, filename):
    u = urllib2.urlopen(url)
    f = open(filename, 'wb')
    meta = u.info()
    file_size = int(meta.getheaders("Content-Length")[0])
    print "Downloading: %s (%s MB)" % (filename, file_size/1000000.0)

    file_size_dl = 0
    block_sz = 65536
    while True:
        buffer = u.read(block_sz)
        if not buffer:
            break

        file_size_dl += len(buffer)
        f.write(buffer)
        status = r"%10d  [%3.2f%%]" % (file_size_dl/1000000.0, file_size_dl * 100. / file_size)
        status = status + chr(8)*(len(status)+1)
        print status,
    f.close()

def tgz_url(object, type):
    if type in ["berkeley_rgbd", "berkeley_rgb_highres"]:
        return base_url + "berkeley/{object}/{object}_{type}.tgz".format(object=object,type=type)
    elif type in ["berkeley_processed"]:
        return base_url + "berkeley/{object}/{object}_berkeley_meshes.tgz".format(object=object,type=type)
    else:
        return base_url + "google/{object}_{type}.tgz".format(object=object,type=type)

def extract_tgz(filename, dir):
    tar_command = "tar -xzf {filename} -C {dir}".format(filename=filename,dir=dir)
    os.system(tar_command)
    os.remove(filename)

def check_url(url):
    try:
        request = urllib2.Request(url)
        request.get_method = lambda : 'HEAD'
        response = urllib2.urlopen(request)
        return True
    except Exception as e:
        return False

def generate_urdf(object_name, data_type, object_mass, data_path):
    if os.access(data_path+"/urdf", os.F_OK) == False:
        try:
            os.mkdir(data_path+ "/urdf")
        except OSError, mkdir_error:
            print "[ycb_benchmarks] Failed to create folder %s. Error message: %s" % (data_path+"/urdf", str(mkdir_error))
            exit(1)

    f = open(data_path + "/urdf/" + object_name + "_" + data_type +".urdf", 'w')
    urdf_str = """
<robot name=\"""" + object_name + """\">
  <link name=\"""" + object_name + """_link">
    <inertial>
      <origin xyz="0 0 0" />
      <mass value=\"""" + str(object_mass) + """\" />
      <inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\"/>
      <geometry>
        <mesh filename=\"""" + """../""" + data_type + """/nontextured.stl\" />
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\"/>
      <geometry>
        <mesh filename=\"""" + """../""" + data_type + """/nontextured.stl\" />
      </geometry>
    </collision>
  </link>
  <gazebo reference=\"""" + object_name + """\">
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
"""
    f.write(urdf_str)
    f.close()

if __name__ == "__main__":

    objects = objects_to_download#fetch_objects(objects_url)

    for object in objects:
        if objects_to_download == "all" or object in objects_to_download:
            for file_type in files_to_download:
                url = tgz_url(object, file_type)
                if not check_url(url):
                    continue
                filename = "{path}/{object}_{file_type}.tgz".format(path=output_directory,
                                                                    object=object,
                                                                    file_type=file_type)
                download_file(url, filename)
                if extract:
                    extract_tgz(filename, output_directory)
                if urdf:
                    generate_urdf(object, "tsdf", object_masses[object], "{path}/{object}".format(path=output_directory,
                                                                    object=object))
