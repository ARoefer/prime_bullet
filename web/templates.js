var template_rb_form = `<h4>Shape</h4> \
          <label for="geom_type">Geometry</label> \
          <select id="rigidGeom" name="geom_type" onchange="update_rigid_form(\'{}\')"> \
            <option value="box">Box</option> \
            <option value="sphere">Sphere</option> \
            <option value="cylinder">Cylinder</option> \
            <option value="capsule">Capsule</option> \
          </select><br> \
          <label for="x" class="boxThings">Depth</label> \
          <input type="number" step="0.0001" name="x" min="0" value="{}" class="boxThings"><br lass="boxThings"> \
          <label for="y" class="boxThings">Width</label> \
          <input type="number" step="0.0001" name="y" min="0" value="{}" class="boxThings"><br lass="boxThings"> \
          <label for="z" class="boxThings cylinderThings">Height</label> \
          <input type="number" step="0.0001" name="z" min="0" value="{}" class="boxThings ylinderThings"><br class="boxThings cylinderThings"> \
           \
          <label for="radius" class="sphereThings" style="display:none">Radius</label> \
          <input type="number" step="0.0001" name="radius" min="0" value="{}" style="display:none" class="sphereThings"><br class="sphereThings"> \
          <label for="mass">Mass</label> \
          <input type="number" step="0.0001" name="mass" value="{}" min="0"><br> \
          <h4>Position</h4> \
          <label for="position_x">X</label> \
          <input type="number" step="0.0001" name="position_x" value="{}"><br> \
          <label for="position_y">Y</label> \
          <input type="number" step="0.0001" name="position_y" value="{}"><br> \
          <label for="position_z">Z</label> \
          <input type="number" step="0.0001" name="position_z" value="{}"><br> \
          <label for="color">Color</label> \
          <input type="color" value="{}"><br>`;

var template_obj_accordion =
        `<button class="objectReference accordion" id="{}">{}</button>
        <div class="panel objectReference">
        <button class="iconBtn objBtn btnDelete" onclick="delete_object('{}')"><i class="fas fa-trash"></i></button>
        <button class="iconBtn objBtn btnReset"  onclick="reset_object('{}')"><i class="fas fa-backward"></i></button>
        <form id="{}">{}</form>
        {}
        </div>`;

var template_mb_form = `<label for="urdf_path">URDF Path</label>
        <input name="urdf_path" type="text" placeholder="package://robot_pkg/robot.urdf" value="{}"><br>
        <!-- <label for="fixed">Fixed Base</label>
        <input type="checkbox" name="fixed" value="false" checked><br> -->
        <h4>Position</h4>
        <label for="position_x">X</label>
        <input type="number" step="0.0001" name="position_x" value="{}"><br>
        <label for="position_y">Y</label>
        <input type="number" step="0.0001" name="position_y" value="{}"><br>
        <label for="position_z">Z</label>
        <input type="number" step="0.0001" name="position_z" value="{}"><br>`;

var template_joint_button = `<button id="{}__{}" class="jointBtn {}" onclick="toggle_joint_sensor('{}', '{}')">
                              <i class="fas fa-eye"></i>
                              <span>{}</span>
                            </button>`;

function generate_rigidbody_accordion(id, name, form_id, rigidbody) {
  return template_obj_accordion.format(id,
                                       name,
                                       name,
                                       name,
                                       form_id,
                                       create_rigid_body_form_content(rigidbody, form_id)
                                      );
}


function generate_multibody_accordion(id, name, form_id, multibody) {
  return template_obj_accordion.format(id,
                                        name,
                                        name,
                                        name,
                                        form_id,
                                        create_multi_body_form_content(
                                          name,
                                          multibody),
                                        '<h4>Joints</h4><div class="jointList">{}</div>'.format(
                                          generate_joint_buttons(
                                              name,
                                              multibody
                                        )));
}


function create_rigid_body_form_content(body, form_id) {
  return template_rb_form.format(
            form_id,
            body.extents.x,
            body.extents.y,
            body.extents.z,
            body.radius,
            body.mass,
            body.pose.position.x,
            body.pose.position.y,
            body.pose.position.z,
            body.color
          );
}


function create_multi_body_form_content(bodyId, body) {
  return template_mb_form.format(
            body.urdf_file,
            body.initial_pose.position.x,
            body.initial_pose.position.y,
            body.initial_pose.position.z,
            generate_joint_buttons(bodyId, body));
}

function generate_joint_buttons(bodyId, body) {
  var out = '';
  var sorted_joints = body.joints.sort();
  for (var j in sorted_joints) {
    out += template_joint_button.format(
            bodyId,
            sorted_joints[j],
            body.sensors.indexOf(sorted_joints[j]) >= 0? 'sensor' : '',
            bodyId,
            sorted_joints[j],
            sorted_joints[j]);
  }
  return out;
}