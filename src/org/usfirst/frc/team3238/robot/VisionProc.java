package org.usfirst.frc.team3238.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

/**
 * Created by aaron on 1/9/2017.
 */
public class VisionProc implements Runnable
{
    private List<MatOfPoint> contours;
    
    public List<MatOfPoint> getContours()
    {
        if(contours != null)
        {
            return contours;
        } else
        {
            return new List<MatOfPoint>()
            {
                @Override public int size()
                {
                    return 0;
                }
                
                @Override public boolean isEmpty()
                {
                    return false;
                }
                
                @Override public boolean contains(Object o)
                {
                    return false;
                }
                
                @Override public Iterator<MatOfPoint> iterator()
                {
                    return null;
                }
                
                @Override public Object[] toArray()
                {
                    return new Object[0];
                }
                
                @Override public <T> T[] toArray(T[] a)
                {
                    return null;
                }
                
                @Override public boolean add(MatOfPoint matOfPoint)
                {
                    return false;
                }
                
                @Override public boolean remove(Object o)
                {
                    return false;
                }
                
                @Override public boolean containsAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public boolean addAll(
                        Collection<? extends MatOfPoint> c)
                {
                    return false;
                }
                
                @Override public boolean addAll(int index,
                        Collection<? extends MatOfPoint> c)
                {
                    return false;
                }
                
                @Override public boolean removeAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public boolean retainAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public void clear()
                {
                    
                }
                
                @Override public MatOfPoint get(int index)
                {
                    return null;
                }
                
                @Override public MatOfPoint set(int index, MatOfPoint element)
                {
                    return null;
                }
                
                @Override public void add(int index, MatOfPoint element)
                {
                    
                }
                
                @Override public MatOfPoint remove(int index)
                {
                    return null;
                }
                
                @Override public int indexOf(Object o)
                {
                    return 0;
                }
                
                @Override public int lastIndexOf(Object o)
                {
                    return 0;
                }
                
                @Override public ListIterator<MatOfPoint> listIterator()
                {
                    return null;
                }
                
                @Override public ListIterator<MatOfPoint> listIterator(
                        int index)
                {
                    return null;
                }
                
                @Override public List<MatOfPoint> subList(int fromIndex,
                        int toIndex)
                {
                    return null;
                }
            };
        }
    }
    
    @Override public void run()
    {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(
                Preferences.getInstance().getInt("Camera width", 640),
                Preferences.getInstance().getInt("Camera height", 480));
        
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance()
                .putVideo("Blur", 640, 480);
        
        Mat source = new Mat();
        Mat proc = new Mat();
        
        while(true)
        {
            cvSink.grabFrame(source);
            Imgproc.cvtColor(source, proc, Imgproc.COLOR_BGR2HSV);
            Core.inRange(proc,
                    new Scalar(Preferences.getInstance().getInt("Lower H", 0),
                            Preferences.getInstance().getInt("Lower S", 0),
                            Preferences.getInstance().getInt("Lower V", 0)),
                    new Scalar(Preferences.getInstance().getInt("Upper H", 180),
                            Preferences.getInstance().getInt("Upper S", 255),
                            Preferences.getInstance().getInt("Upper V", 255)),
                    proc);
            if(proc.type() != CvType.CV_8UC1)
            {
                proc.convertTo(proc, CvType.CV_8UC1);
            }
            Imgproc.findContours(proc, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_KCOS);
            Imgproc.drawContours(source, contours, -1, new Scalar(255, 0, 0));
            outputStream.putFrame(source);
        }
    }
}
