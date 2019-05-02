within GIT_thesis_Bram_Stockman;
model Unnamed1
  Modelica.Blocks.Sources.Constant const
    annotation (Placement(transformation(extent={{-66,22},{-46,42}})));
  Modelica.Blocks.Sources.Constant const1(k=2)
    annotation (Placement(transformation(extent={{-66,-20},{-46,0}})));
equation
  connect(unnamed.Go, const.y) annotation (Line(points={{-15.8,12.2},{-30,
          12.2},{-30,32},{-45,32}}, color={0,0,127}));
  connect(const1.y, unnamed.u1) annotation (Line(points={{-45,-10},{-28,-10},
          {-28,7.2},{-15.8,7.2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Unnamed1;
