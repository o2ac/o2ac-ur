#!/usr/bin/python
import csv
import os

import rospy
import rospkg
import tf
import geometry_msgs
from math import pi
rp = rospkg.RosPack()

# Read in the assembly matings
mating_filename = os.path.join(
    rp.get_path("o2ac_parts_description"),
    "config",
    "frames_to_mate.csv")
frame_matings = []
with open(mating_filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if row:     # Skips empty rows
            if row[0][0] == "#":    # Skips commented rows
                continue
            row_stripped = []
            for el in row:
                row_stripped.append(el.strip())       # Removes whitespaces
            frame_matings.append(row_stripped)

frames_filename = os.path.join(
    rp.get_path("o2ac_parts_description"),
    "config",
    "extra_frames.csv")
extra_frames = []
with open(frames_filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if row:     # Skips empty rows
            if row[0][0] == "#":    # Skips commented rows
                continue
            row_stripped = []
            for el in row:
                row_stripped.append(el.strip())       # Removes whitespaces
            extra_frames.append(row_stripped)

template_filename = os.path.join(
    rp.get_path("o2ac_parts_description"),
    "urdf/templates",
    "assembly_template.urdf.xacro")
f = open(template_filename, 'r')
template_front = f.read()
f.close()

# Write the file containing connections between the frames in the instructions
out_dir = os.path.join(rp.get_path("o2ac_parts_description"), "urdf/generated")
outfile = open(os.path.join(out_dir, "full_assembly.urdf.xacro"), 'w+')
content = template_front

content += "    <xacro:assy_part_01 prefix=\"${prefix}\" parent=\"${parent}\" spawn_attached=\"true\"> \n"
content += "      <xacro:insert_block name=\"origin\"/> \n"
content += "    </xacro:assy_part_01> \n"
content += "    \n"
for mating in frame_matings:
    parent_frame = mating[0]
    child_frame = mating[1]
    parent_part_num = int(parent_frame[10:12])
    child_part_num = int(child_frame[10:12])
    child_part_base_frame = "assy_part_" + str(child_part_num).zfill(2)
    child_part_subframe_name = child_frame[13:]

    # Find the name of the child_frame in the extra_frames, and use its offset to link it to that part's base frame.
    # This is necessary because only one frame in the tree may be without a
    # parent.
    for subframe_offset in extra_frames:
        if subframe_offset[1] == child_part_subframe_name and int(
                subframe_offset[0]) == child_part_num:
            subframe_offset = subframe_offset
            break

    # Get the inverse transform for the transformation entered into extra_frames
    # The transformation has to be set manually because this file is executed before the robot is started up,
    # so TF does not publish these frames.
    rpy = [
        float(
            eval(
                subframe_offset[2])), float(
            eval(
                subframe_offset[3])), float(
            eval(
                subframe_offset[4]))]
    xyz = [
        float(
            eval(
                subframe_offset[5])), float(
            eval(
                subframe_offset[6])), float(
            eval(
                subframe_offset[7]))]
    q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

    t = tf.TransformerROS(True, rospy.Duration(10.0))
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = child_part_base_frame
    m.child_frame_id = child_frame
    m.transform.translation.x = xyz[0]
    m.transform.translation.y = xyz[1]
    m.transform.translation.z = xyz[2]
    m.transform.rotation.x = q[0]
    m.transform.rotation.y = q[1]
    m.transform.rotation.z = q[2]
    m.transform.rotation.w = q[3]
    t.setTransform(m)

    m2 = geometry_msgs.msg.TransformStamped()
    m2.header.frame_id = child_frame
    m2.child_frame_id = "mating_position"
    m2.transform.translation.x = float(eval(mating[5]))
    m2.transform.translation.y = float(eval(mating[6]))
    m2.transform.translation.z = float(eval(mating[7]))
    q = tf.transformations.quaternion_from_euler(
        float(
            eval(
                mating[2])), float(
            eval(
                mating[3])), float(
            eval(
                mating[4])))
    m2.transform.rotation.x = q[0]
    m2.transform.rotation.y = q[1]
    m2.transform.rotation.z = q[2]
    m2.transform.rotation.w = q[3]
    t.setTransform(m2)

    t_inv, q_i = t.lookupTransform(
        "mating_position", child_part_base_frame, rospy.Time(0))
    rpy_inv = tf.transformations.euler_from_quaternion(q_i)

    # First, spawn the part unattached (no link to the world). Then, create
    # the mating joint manually, offset to the part's base frame.
    new_mating = ""
    new_mating += "    <xacro:assy_part_" + str(child_part_num).zfill(
        2) + " prefix=\"${prefix}\" parent=\"${prefix}" + parent_frame + "\" spawn_attached=\"false\"> \n"
    new_mating += "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/> \n"
    new_mating += "    </xacro:assy_part_" + \
        str(child_part_num).zfill(2) + "> \n"
    new_mating += "    \n"
    new_mating += "    <joint name=\"${prefix}part_" + str(parent_part_num).zfill(
        2) + "_to_" + str(child_part_num).zfill(2) + "_joint\" type=\"fixed\"> \n"
    new_mating += "      <parent link=\"${prefix}" + parent_frame + "\"/> \n"
    new_mating += "      <child link=\"${prefix}" + \
        child_part_base_frame + "\"/> \n"
    new_mating += "      <origin rpy=\"${" + \
        str(rpy_inv[0]) + "} ${" + \
        str(rpy_inv[1]) + "} ${" + \
        str(rpy_inv[2]) + "}\" xyz=\"${" + \
        str(t_inv[0]) + "} ${" + \
        str(t_inv[1]) + "} ${" + \
        str(t_inv[2]) + "}\"/> \n"
    new_mating += "    </joint> \n"
    new_mating += "    \n"
    content += new_mating

content += "  </xacro:macro>\n"
content += "</robot>\n"
outfile.write(content)


# Write the same assembly, but without the collision and visual bodies
content = content.replace("xacro:assy_part_", "xacro:assy_part_frames_only_")
content = content.replace(
    "name=\"full_assembly\"",
    "name=\"full_assembly_frames_only\"")
content = content.replace(
    "assembly_parts_macros.urdf.xacro",
    "assembly_parts_frame_macros.urdf.xacro")

out_dir2 = os.path.join(
    rp.get_path("o2ac_parts_description"),
    "urdf/generated")
outfile2 = open(
    os.path.join(
        out_dir2,
        "full_assembly_frames_only.urdf.xacro"),
    'w+')
outfile2.write(content)
