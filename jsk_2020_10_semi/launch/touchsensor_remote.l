<launch>
  <node pkg="rosserial_python" type="serial_node.py" name="rosserial_python_node"
         />
  <node pkg="rostopic" type="rostopic" name="listener"
        args="echo /pressure" />
</launch>
