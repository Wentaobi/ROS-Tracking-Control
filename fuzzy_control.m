function fuzzy_control
    clear
    global oa;
    global fbs;
    global target_ob;
    %setup desired goal
    target_ob=[-7,7];
    %initial fuzzy lab
    oa=readfis('oa');
    fbs=readfis('fbs');
    global node;
    global vd;
    global wd;
    global k2;
    global k3;
    %setup factor in Neural controller
    vd=2;
    wd=2;
    k2=2;
    k3=4;
    %setup ROS communication
    %setenv('ROS_MASTER_URI','http://172.18.50.82:11311')
    %setenv('ROS_IP','172.18.50.82')
	setenv('ROS_MASTER_URI','http://192.168.12.42:11311')
	setenv('ROS_IP','192.168.12.42')
    node=rosmatlab.node('fuzzy_control','http://192.168.12.42:11311');
    %publish velocity
    publisher = rosmatlab.publisher('/cmd_vel', 'geometry_msgs/Twist', node);
    %subscribe position and scan information
    subScan=rosmatlab.subscriber('/command','std_msgs/Float64MultiArray',1,node);
    subScan.setOnNewMessageListeners({@ScanGet});
    %processing function
    function ScanGet(msgs)
        %receive data
        comm=msgs.getData;
        d2=comm(1);
        d3=comm(2);
        d4=comm(3);
        odom_x=comm(4);
        odom_y=comm(5);
        odom_a=comm(6);
        %setup scan array
        scan=[d2 d3 d4];
        %setup maximum limit for input of OA
        if(d2>=5)
            d2=4.9;
        end
        if(d3>=5)
            d3=4.9;
        end
        if(d4>=5)
            d4=4.9;
        end
        %set input array for OA
        in=[d2 d3 d4];
        %OA fuzzy processing
        oa_out=evalfis(in,oa);
        %%calculate angel between goal and robot
        doa=atan2(target_ob(2)-odom_y,target_ob(1)-odom_x)-odom_a;
        %make angle in range (-pi, pi)
        while (doa>pi)||(doa<-pi)
            if(doa>pi)
                doa=doa-2*pi;
            end
            if(doa<-pi)
                doa=doa+2*pi;
            end
        end
        %Neural tracking control
        %calculate desired angle for neural control
        prd_a=atan2(target_ob(2)-odom_y,target_ob(1)-odom_x);
        %calculate intpu error
        error_l=-(target_ob(1)-odom_x)*sin(odom_a)+(target_ob(2)-odom_y)*cos(odom_a);
        error_a=prd_a-odom_a;
        %velocity calculate
        vc=oa_out(1)*cos(error_a);
        wc=oa_out(2)+k2*oa_out(1)*error_l+k3*oa_out(1)*sin(error_a);
        %calculate input of the 
        dmin=min(in);
        dot=ceil((doa+1.5*pi/2)/(1.5*pi/5));
        %weight caluclate input setting
        fbs_in=[dmin,scan(dot-1)];
        %weight calculate fuzzy processing
        fbs_out=evalfis(fbs_in,fbs);
        %calculate distance towards goal
        dt=((odom_x-target_ob(1))^2+(odom_y-target_ob(2))^2)^0.5;
        %if achieve goal
        if(dt>1)
            if(fbs_out<0.5) 
                w=oa_out(2);
                v=oa_out(1);
            else
                w=-wc;
                v=vc;
            end
        else
            v=0;
            w=0;
        end
        %ros publish
        msg = rosmatlab.message('geometry_msgs/Twist', node);
        msgLin = rosmatlab.message('geometry_msgs/Vector3', node);
        msgAng = rosmatlab.message('geometry_msgs/Vector3', node);
        msgLin.setX(v);
        msgLin.setY(0);
        msgLin.setZ(0);
        msg.setLinear (msgLin);
        msgAng.setX(0);
        msgAng.setY(0);
        msgAng.setZ(-w);
        msg.setAngular (msgAng);
        publisher.publish(msg);
    end
    %waiting
    str = input('Enter any key to terminate','s');
end
