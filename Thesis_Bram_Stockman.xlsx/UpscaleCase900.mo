within GIT_thesis_Bram_Stockman;
model UpscaleCase900 "Upscaling of the case 900"
  extends IDEAS.Airflow.Multizone.Structure(
    final nZones=1,
    nEmb=1,
    ATrans=1,
    VZones={gF.V});
  constant Modelica.SIunits.Angle aO = 0 "Angle offset for detailed experiments";

  IDEAS.Buildings.Components.Zone gF(
    mSenFac=0.822,
    V=1200*2.7,
    n50=0.822*0.5*20,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2.7,
    T_start=295.15)
                annotation (Placement(transformation(extent={{40,0},{80,40}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{120,-70},{140,-50}})));
  IDEAS.Buildings.Components.OuterWall[4] wall(
    redeclare parameter IDEAS.Buildings.Validation.Data.Constructions.HeavyWall constructionType,
    A={108,81,48,81},
    final azi={aO+IDEAS.Types.Azimuth.N,aO+IDEAS.Types.Azimuth.E,aO+IDEAS.Types.Azimuth.S,
        aO+IDEAS.Types.Azimuth.W},
    final inc={IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall,
        IDEAS.Types.Tilt.Wall}) annotation (Placement(transformation(
        extent={{-5.5,-9.49999},{5.5,9.49999}},
        rotation=90,
        origin={-49.5,-14.5})));

  IDEAS.Buildings.Components.Window[2] win(
    final A={30,30},
    redeclare final parameter IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazing,
    final inc={IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall},
    azi={aO+IDEAS.Types.Azimuth.N,aO+IDEAS.Types.Azimuth.N},
    redeclare replaceable IDEAS.Buildings.Components.Shading.None shaType,
    redeclare final parameter IDEAS.Buildings.Data.Frames.None fraType,
    each frac=0)
    annotation (Placement(transformation(
        extent={{-5.5,-9.49999},{5.5,9.49997}},
        rotation=90,
        origin={10.5,-14.5})));

  IDEAS.Buildings.Components.BoundaryWall floor(
    redeclare parameter IDEAS.Buildings.Data.Constructions.InsulatedFloorHeating constructionType,
    final A=1200,
    inc=IDEAS.Types.Tilt.Floor,
    final azi=aO+IDEAS.Types.Azimuth.S) annotation (Placement(transformation(
        extent={{-5.5,-9.5},{5.5,9.5}},
        rotation=90,
        origin={-19.5,-14.5})));
  IDEAS.Buildings.Components.OuterWall roof(
    final A=1200,
    final inc=IDEAS.Types.Tilt.Ceiling,
    final azi=aO+IDEAS.Types.Azimuth.S,
    redeclare final parameter
      IDEAS.Buildings.Validation.Data.Constructions.LightRoof
      constructionType)                 annotation (Placement(transformation(
        extent={{-5.5,-9.5},{5.5,9.5}},
        rotation=90,
        origin={-79.5,-14.5})));

equation
  connect(temperatureSensor.T, TSensor[1]) annotation (Line(
      points={{140,-60},{156,-60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(gF.gainCon, temperatureSensor.port) annotation (Line(
      points={{80,14},{100,14},{100,-60},{120,-60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(gF.gainCon, heatPortCon[1]) annotation (Line(
      points={{80,14},{120,14},{120,20},{150,20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(gF.gainRad, heatPortRad[1]) annotation (Line(
      points={{80,8},{120,8},{120,-20},{150,-20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(win.propsBus_a, gF.propsBus[7:8]) annotation (Line(
      points={{8.60001,-9.91667},{8.60001,24.5},{40,24.5}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));

  connect(roof.propsBus_a, gF.propsBus[1]) annotation (Line(
      points={{-81.4,-9.91667},{-81.4,31.5},{40,31.5}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(wall.propsBus_a, gF.propsBus[2:5]) annotation (Line(
      points={{-51.4,-9.91667},{-51.4,27.5},{40,27.5}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(floor.propsBus_a, gF.propsBus[6]) annotation (Line(
      points={{-21.4,-9.91667},{-21.4,26.5},{40,26.5}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(port_b[1], gF.port_b) annotation (Line(
      points={{-20,100},{-20,60},{56,60},{56,40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(port_a[1], gF.port_a) annotation (Line(
      points={{20,100},{20,62},{64,62},{64,40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(floor.port_emb[1], heatPortEmb[1]) annotation (Line(points={{-10,-14.5},
          {-4,-14.5},{-4,-30},{110,-30},{110,60},{150,60}}, color={191,0,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-150,-100},
            {150,100}}),       graphics), Documentation(info="<html>
<p>
Basic, most generic structure of the BesTest model.
To be extended in other models.
</p>
</html>", revisions="<html>
<ul>
<li>
March 8, 2017 by Filip Jorissen:<br/>
Added angle for offsetting building rotation.
This is for 
<a href=https://github.com/open-ideas/IDEAS/issues/689>#689</a>.
</li>
<li>
July 19, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"));
end UpscaleCase900;
