package lab4_205_01.uwaterloo.ca.lab4_205_01;

import android.graphics.PointF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.ContextMenu;
import android.view.MenuItem;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import ca.uwaterloo.sensortoy.mapper.InterceptPoint;
import ca.uwaterloo.sensortoy.mapper.MapLoader;
import ca.uwaterloo.sensortoy.mapper.MapView;
import ca.uwaterloo.sensortoy.mapper.NavigationalMap;
import ca.uwaterloo.sensortoy.mapper.PositionListener;


//class that implements the step detection and counter
class StepDetector implements SensorEventListener, PositionListener {

    public float[] R = new float[9];
    public float[] I = new float[9];
    public float[] gravity = new float[3];
    public float[] geomagnetic = new float[3];
    public boolean rotationMatrix = false;
    public double slope = 0.0;
    public int state = 0;
    public int step;
    public double stepNCalculated, stepECalculated, stepsN, stepsE;
    public double azimuth;
    public TextView stepView, northView, eastView, azimuthView, instructionsViewE, instructionsViewN;
    public static double smoothedAccel = 0.0, previousSmoothedAccel = 0.0;


    PointF origin, user, destination, prevuser;
    List<PointF> userPath;
    MapView mapView;

    public StepDetector(TextView steps, TextView north, TextView east, TextView azimuth, MapView mv, TextView instructionsE, TextView instructionsN) {
        stepView = steps;
        northView = north;
        eastView = east;
        azimuthView = azimuth;
        mapView = mv;
        instructionsViewE = instructionsE;
        instructionsViewN = instructionsN;

        user = new PointF(4, 4);
        originChanged(mapView, user);

        //start and end points
        origin = new PointF(4, 4);
        destination = new PointF(18, 4);
        destinationChanged(mapView, destination);
        mapView.setOriginPoint(origin);

        //path
        userPath = new ArrayList<>();
        userPath = userPath(origin, destination);
        setInstructions();
        mapView.setUserPath(userPath);
    }

    public void onAccuracyChanged(Sensor s, int i) {
    }

    //grabbing data from the sensor in the y-axis
    public void onSensorChanged(SensorEvent se) {
        if (se.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            previousSmoothedAccel = smoothedAccel;
            //low pass filter to move our acceleration values closer to the new value,
            //with a constant determining the attenuation
            //i.e. dividing by the 1.7 will move our new value 60% of the way
            smoothedAccel += (se.values[1] - smoothedAccel) / 1.7;
            //calculating slope by measuring the difference between current and previous values
            slope = smoothedAccel - previousSmoothedAccel;
            FSM();
        } else if (se.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            for (int i = 0; i < 3; i++) {
                geomagnetic[i] = se.values[i];
            }
        } else if (se.sensor.getType() == Sensor.TYPE_GRAVITY) {
            for (int i = 0; i < 3; i++) {
                gravity[i] = se.values[i];
            }
        }
        //get the rotation matrix
        rotationMatrix = SensorManager.getRotationMatrix(R, I, gravity, geomagnetic);
        if (rotationMatrix) {
            //get rotation and use [0] which is azimuth

            float orientation[] = new float[3];
            SensorManager.getOrientation(R, orientation);

            //Use a baseline as the starting value and move the next readings closer by a factor of 1/3 the distance
            if (azimuth == 0) {
                azimuth = orientation[0];
            } else {
                azimuth += (orientation[0] - azimuth) / 3;
            }
            azimuthView.setText("Azimuth:" + Double.toString(Math.toDegrees(azimuth)));
            //Calculate North and East components using cos and sin functions on the azimuth
            stepNCalculated = Math.cos(azimuth);
            stepECalculated = Math.sin(azimuth);


        }
    }

    //finite state machine, with each state representing a stage in y-acceleration waveform
    public void FSM() {
        //these two if statements are error checks to prevent shaking of device adding to our step counter.
        //i.e. if the change in acceleration > 0.5, we consider that to be an extreme change that is not corresponding to a step
        if (Math.abs(slope) > 0.5) {
            state = 0;
        }
        //for an extremely small change in acceleration, we not consider it to be significant enough to add to our steps
        if (Math.abs(slope) < 0.01) {
            return;
        }
        switch (state) {

            case 0:
                if (smoothedAccel > 0.4) {
                    state++;
                }
                break;
            case 1:
                if (smoothedAccel > 0.5) {
                    state++;
                }
                break;
            case 2:
                if (smoothedAccel > 0.6) {
                    state++;
                }
                break;
            case 3:
                if (smoothedAccel > 0.8) {
                    state++;
                }
                break;
            case 4:
                if (smoothedAccel < 0.6) {
                    state++;
                }
                break;
            case 5:
                if (smoothedAccel < 0.5) {
                    state++;
                }
                break;
            case 6:
                if (smoothedAccel < 0.4) {
                    state = 0;
                    step++;
                    stepsN += stepNCalculated;
                    stepsE += stepECalculated;
                    azimuth = 0;

                    //update userpoint
                    prevuser = new PointF(user.x, user.y);
                    if (Math.abs(stepNCalculated) > Math.abs(stepECalculated)) {
                        if (stepNCalculated > 0) {
                            user.set(user.x, user.y - 1);
                        } else {
                            user.set(user.x, user.y + 1);
                        }
                    } else {
                        if (stepECalculated > 0) {
                            user.set(user.x + 1, user.y);
                        } else {
                            user.set(user.x - 1, user.y);
                        }
                    }

                    if (!mapView.map.calculateIntersections(prevuser, user).isEmpty()) {
                        user.set(prevuser.x, prevuser.y);
                        step--;
                        stepsN -= stepNCalculated;
                        stepsE -= stepECalculated;
                    }
                    //update path
                    if(userPath.get(1).x ==user.x &&userPath.get(1).y ==user.y ) {
                        userPath.remove(0);

                    }else{
                        userPath = userPath(user,destination);

                    }
                    mapView.setUserPath(userPath);
                    //new userpoint
                    originChanged(mapView, user);

                    //Generate instructions
                    if(user.x==destination.x && user.y==destination.y){
                        instructionsViewE.setText("");
                        instructionsViewN.setText("Destination Reached!");
                    }else{
                        setInstructions();
                    }

                    stepView.setText("Steps:" + step);
                    northView.setText("North:" + stepsN);
                    eastView.setText("East:" + stepsE);
                }
                break;


        }
    }

    public void setInstructions(){
        PointF currentp, nextp;
        float Nstep, Estep;
        currentp = userPath.get(0);
        nextp = userPath.get(1);
        Nstep = nextp.y - currentp.y;
        Estep = nextp.x - currentp.x;
        if (Nstep > 0) {
            instructionsViewN.setText("Walk " + (Nstep) + " steps South");
        } else if (Nstep < 0) {
            instructionsViewN.setText("Walk " + (-Nstep) + " steps North");
        } else {
            instructionsViewN.setText("");
        }
        if (Estep < 0) {
            instructionsViewE.setText("Walk " + (-Estep) + " steps West");
        } else if (Estep > 0) {
            instructionsViewE.setText("Walk " + (Estep) + " steps East");
        } else {
            instructionsViewE.setText("");
        }
    }
    //Function for generating a userpath given start and end points
    public List<PointF> userPath(PointF start, PointF end) {
        List<PointF> userPath = new ArrayList<>();
        List<InterceptPoint> interceptPoints;

        userPath.add(start);
        PointF currentPoint = new PointF(start.x, start.y);
        PointF prevPoint = new PointF(start.x, start.y);
        //Algorithm for determining path
        interceptPoints = mapView.map.calculateIntersections(start, end);
        int counter = 1;
        int stepx =(int)((end.x-start.x)/(Math.abs(end.x-start.x)));
        int stepy =1;
        while (!interceptPoints.isEmpty()) {
            prevPoint.set(currentPoint.x, currentPoint.y);
            currentPoint.set(prevPoint.x + stepx, prevPoint.y);
            if (!mapView.map.calculateIntersections(prevPoint, currentPoint).isEmpty()) {

                currentPoint.set(prevPoint.x, prevPoint.y + stepy);
                if (!mapView.map.calculateIntersections(prevPoint, currentPoint).isEmpty()) {
                    currentPoint.set(prevPoint.x, prevPoint.y);
                    stepy*=-1;

                }
            }
            System.out.println("x: " + currentPoint.x + " y: " + currentPoint.y);
            if(currentPoint.x != prevPoint.x || currentPoint.y != prevPoint.y) {
                userPath.add(new PointF(currentPoint.x, currentPoint.y));
            }

            interceptPoints = mapView.map.calculateIntersections(currentPoint, end);
            counter++;
            if (counter > 400) {
                break;
            }
        }


        userPath.add(end);




        System.out.println(userPath.size());
        return userPath;
    }

    @Override
    public void originChanged(MapView source, PointF loc) {
        source.setUserPoint(loc);
    }

    @Override
    public void destinationChanged(MapView source, PointF dest) {
        source.setDestinationPoint(dest);
    }
}

public class Lab4_205_01 extends AppCompatActivity {
    SensorEventListener sensorListener;
    SensorManager sensorManager;
    Sensor accelerateSensor, magneticfieldSensor, gravitySensor;
    TextView steps, east, north, azimuth, instructionsE, instructionsN;
    MapView mapView;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_lab4_205_01);

        //Map View
        mapView = new MapView(getApplicationContext(), 1000, 1000, 35, 35);
        registerForContextMenu(findViewById(R.id.scroll));

        NavigationalMap map = MapLoader.loadMap(getExternalFilesDir(null), "E2-3344.svg");
        mapView.setMap(map);


        LinearLayout layout = (LinearLayout) findViewById(R.id.main_page);
        layout.setOrientation(LinearLayout.VERTICAL);


        //add the map and set origin
        layout.addView(mapView);


        //get the text view from layout
        steps = (TextView) findViewById(R.id.stepsView_Current);
        north = (TextView) findViewById(R.id.northDistance);
        east = (TextView) findViewById(R.id.eastDistance);
        azimuth = (TextView) findViewById(R.id.azimuth);
        instructionsE = (TextView) findViewById(R.id.instructionsE);
        instructionsN = (TextView) findViewById(R.id.instructionsN);
        //get the sensor manager
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerateSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        magneticfieldSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        //set the sensoreventlistener

        sensorListener = new StepDetector(steps, north, east, azimuth, mapView, instructionsE, instructionsN);

        //register listener
        sensorManager.registerListener(sensorListener, accelerateSensor, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorListener, magneticfieldSensor, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(sensorListener, gravitySensor, SensorManager.SENSOR_DELAY_NORMAL);
        //button


    }

    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenu.ContextMenuInfo menuInfo) {
        super.onCreateContextMenu(menu, v, menuInfo);
        mapView.onCreateContextMenu(menu, v, menuInfo);
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return super.onContextItemSelected(item) || mapView.onContextItemSelected(item);
    }

    //button is pressed
    public void resetSteps(View view) {
        sensorManager.unregisterListener(sensorListener);
        steps.setText("Steps:0");
        east.setText("East:0");
        north.setText("North:0");
        sensorListener = new StepDetector(steps, north, east, azimuth, mapView, instructionsE, instructionsN);
        //register listener
        sensorManager.registerListener(sensorListener, accelerateSensor, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorListener, magneticfieldSensor, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(sensorListener, gravitySensor, SensorManager.SENSOR_DELAY_NORMAL);

    }


}



