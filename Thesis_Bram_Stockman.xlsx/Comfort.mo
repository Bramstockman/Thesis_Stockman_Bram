within GIT_thesis_Bram_Stockman;
model Comfort

  IDEAS.Controls.SetPoints.Table LowerComfortCurve(table=[20 + 273,20 + 273;
        22 + 273,22 + 273])
    annotation (Placement(transformation(extent={{-36,22},{-16,42}})));
  IDEAS.Controls.SetPoints.Table UpperComfortCurve(table=[20 + 273,24 + 273; 23
         + 273,26 + 273])
    annotation (Placement(transformation(extent={{-36,-14},{-16,6}})));
  Modelica.Blocks.Interfaces.RealOutput LowerComfort
    annotation (Placement(transformation(extent={{100,22},{120,42}})));
  Modelica.Blocks.Interfaces.RealOutput UpperComfort
    annotation (Placement(transformation(extent={{100,-14},{120,6}})));

  Outside_temperature                          outside_temperature
    annotation (Placement(transformation(extent={{-78,0},{-58,20}})));
  IDEAS.Controls.SetPoints.OccupancySchedule occSch(
    period=604800,
    firstEntryOccupied=true,
    occupancy=3600*{7,21,31,45,55,69,79,93,103,117,127,141,151,165})
    annotation (Placement(transformation(extent={{-140,-4},{-120,16}})));
  Modelica.Blocks.Math.Add LowerComfortDayNight(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Modelica.Blocks.Math.Add UppercomfortDayNight(k1=+1, k2=+1)
    annotation (Placement(transformation(extent={{18,-30},{38,-10}})));
  Modelica.Blocks.Logical.Switch switch2
    annotation (Placement(transformation(extent={{-40,-56},{-20,-36}})));
  Modelica.Blocks.Sources.Constant const(k=5)
    annotation (Placement(transformation(extent={{-78,48},{-58,68}})));
  Modelica.Blocks.Sources.Constant const1(k=0)
    annotation (Placement(transformation(extent={{-78,82},{-58,102}})));
  Modelica.Blocks.Sources.Constant const2(k=5)
    annotation (Placement(transformation(extent={{-78,-78},{-58,-58}})));
  Modelica.Blocks.Sources.Constant const3(k=0)
    annotation (Placement(transformation(extent={{-78,-40},{-58,-20}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{-40,66},{-20,86}})));

equation
  connect(occSch.occupied, switch1.u2) annotation (Line(points={{-119,0},{-98,0},
          {-98,76},{-42,76}}, color={255,0,255}));
  connect(switch2.u2, switch1.u2) annotation (Line(points={{-42,-46},{-98,-46},{
          -98,76},{-42,76}}, color={255,0,255}));
  connect(const1.y, switch1.u1) annotation (Line(points={{-57,92},{-50,92},{-50,
          84},{-42,84}}, color={0,0,127}));
  connect(const.y, switch1.u3) annotation (Line(points={{-57,58},{-50,58},{-50,68},
          {-42,68}}, color={0,0,127}));
  connect(const3.y, switch2.u1) annotation (Line(points={{-57,-30},{-48,-30},{-48,
          -38},{-42,-38}}, color={0,0,127}));
  connect(const2.y, switch2.u3) annotation (Line(points={{-57,-68},{-50,-68},{-50,
          -54},{-42,-54}}, color={0,0,127}));
  connect(outside_temperature.TOutside, LowerComfortCurve.u) annotation (Line(
        points={{-57,10.6},{-57,10},{-56,10},{-56,32},{-38,32}}, color={0,0,127}));
  connect(outside_temperature.TOutside, UpperComfortCurve.u) annotation (Line(
        points={{-57,10.6},{-56,10.6},{-56,-4},{-38,-4}}, color={0,0,127}));
  connect(LowerComfortCurve.y, LowerComfortDayNight.u2) annotation (Line(points=
         {{-15,32},{2,32},{2,44},{18,44}}, color={0,0,127}));
  connect(switch1.y, LowerComfortDayNight.u1) annotation (Line(points={{-19,76},
          {0,76},{0,56},{18,56}}, color={0,0,127}));
  connect(UpperComfortCurve.y, UppercomfortDayNight.u1) annotation (Line(points={{-15,-4},
          {0,-4},{0,-14},{16,-14}},          color={0,0,127}));
  connect(switch2.y, UppercomfortDayNight.u2) annotation (Line(points={{-19,-46},
          {0,-46},{0,-26},{16,-26}}, color={0,0,127}));
  connect(LowerComfortDayNight.y, LowerComfort) annotation (Line(points={{41,50},
          {72,50},{72,32},{110,32}}, color={0,0,127}));
  connect(UppercomfortDayNight.y, UpperComfort) annotation (Line(points={{39,-20},
          {72,-20},{72,-4},{110,-4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},
            {100,120}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{100,120}})),
                Line(points={{-15,32},{110,32}}, color={0,0,127}),
                Line(points={{110,32},{110,32}}, color={0,0,127}),
                Line(points={{-15,-4},{110,-4}}, color={0,0,127}));
end Comfort;
