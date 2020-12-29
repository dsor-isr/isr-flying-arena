visualization_tool(["2020_10_01_08h_48_uav1.csv", "2020_10_01_08h_48_uav2.csv"], 20, 20, 'matlab_post_visualization.avi', 30)





function visualization_tool(logs, azimuth, elevation, video_name, frame_rate)
%
%   Reproduces, through the log files, the evolution of the position and attitude 
%   of the drones of the experiment. Records a video with the generated frames.
%
%   visualization_tool(logs, azimuth, elevation, video_name, frame_rate)
%
%   logs        --> string array with the name of all the log files.
%   azimuth     --> azimuth of the desired 3D view.
%   elevation   --> elevation of the desired 3D view.
%   video_name  --> name of the video file created through the generated frames.
%   frame_rate  --> rate of video playback.

    global p;
    global q;
    
    set(gca, 'nextplot', 'replacechildren');
    v = VideoWriter(video_name, 'Motion JPEG AVI');
    v.FrameRate = frame_rate;
    v.Quality = 100;
    open(v)
   
    load_pose_from_logs(logs);
    [iter, ~, num_drones] = size(p);

    for i = 1:iter
        clf;
        for j = 1:num_drones
            pij = p(i,:,j);
            pij = pij(:,[2 1 3]);    
            qij = q(i,:,j);
            qij = qij(:,[2 1 3]);
            quad_plot(pij, eul2rotm(qij, 'XYZ'), 1, [], [], 0.35);
        end
        axis([-3 3 -4 4 -0.5 2.8]);
        xlabel('Position East [m]');
        ylabel('Position North [m]');
        zlabel('-Position Down [m]');
        view(azimuth, elevation)
        if i>=5
            frame=getframe(gcf);
            writeVideo(v, frame);
        end
        pause(1/100);
    end

    close(v);

end





function load_pose_from_logs(logs)
%
%   Retrieves, from the log file, the set of positions and orientations
%   adopted by the vehicle during the flight. 
%
%   load_pose_from_logs(logs)
%
%   logs        --> string with the name of the log file.

    global p;
    global q;

    num_drones = length(logs);
    for i = 1:num_drones
        log_data = readtable(logs(i));
        p(:,:,i) = table2array(log_data(:, 2:4)).*[1 1 -1];
        q(:,:,i) = table2array(log_data(:, 15:17));
    end

end 





function quad_plot(position, rot, auto, maincolor, alpha, esc)
%
%   Plots a quadrotor from position and rotation matrix orientation.
%
%   quad_plot(position, rot, auto, maincolor, alpha, esc)
%
%   general function to plot a quadrotor with given "position" vector and
%   direct Cosine "rotation" matrix. The function is made such that when saving
%   the Matlab image in eps, the result is a truly vectorial image (since
%   some drawing functions make this impossible by saving the eps images
%   as bitmaps).
%
%   position    --> position vector [x;y;z].
%   rot         --> Rotation Matrix
%   auto        --> plots heli in the active figure or in an independent one:
%                   0 -> assuming that "hold" is on;
%                   1 -> erases active figure and plots the helicopter.
%   maincolor   --> specify RGB color of the helicopter (the default is
%                   yellow, when "color = []").
%   alpha       --> Transparency (0 - transparent, 1 - opaque)
%                    (the default is opaque, when "alpha = []").
%   esc         --> Scaling (1 is approx. 1 meter radius)

    if isempty(position)
        position = [0 0 0];
    end
    
    if isempty(rot)
        rot = eye(3);
    end

    if isempty(auto)
        auto = 1;
    end

    if isempty(maincolor)
        maincolor = [0.95 0.95 0.0];
    end

    if isempty(alpha)
        alpha = 1;
    end

    if isempty(esc)
        esc = 1;
    end


    % Define colors
    LW = 0.05;
    mainbarcolor = [1 0 0];
    barcolor = [0.1 0.1 0.1];
    propcolor = [.9 .9 .9];
    edgecolor = [0 0 0];

    % Define dimensions
    rc = 0.4;
    rp = .2;
    l = .9;
    h = 0.4;
    w = 0.05;

    if auto
        hold on;
    end

    % Central body
    [x y z] = cylinder;
    xbodytmp = rc*x;
    ybodytmp = rc*y;
    zbodytmp = h/1.5*z-h/3;

    xbody = zeros(size(xbodytmp));
    ybody = xbody;
    zbody = xbody;

    % propeller at x positive
    xproptmp = rp*x + l;
    yproptmp = rp*y;
    zproptmp = w*z/10+h/2-w/2;

    xprop = zeros(size(xproptmp));
    yprop = xprop;
    zprop = xprop;

    for i=1:2
        body = (rot*([xbodytmp(i,:)' ybodytmp(i,:)' zbodytmp(i,:)']*esc)')' ...
            + kron(ones(length(xbodytmp),1),position);

        xbody(i,:) = body(:,1);
        ybody(i,:) = body(:,2);
        zbody(i,:) = body(:,3);
    end

    % main bar
    vert_bar_xpos = [2*rp w/2 w/2 %1
        2*rp w/2 -w/2
        2*rp -w/2 w/2 % 3
        2*rp -w/2 -w/2
        l w/2 w/2 % 5
        l w/2 -w/2
        l -w/2 w/2 % 7
        l -w/2 -w/2];

    face_bar_xpos = [  1  2  4 3
        5  6  8  7
        1  2  6  5
        1  3  7  5
        4  2  6  8
        4  3  7  8 ];

    % propeller bar
    vert_propbar_xpos = [l-w/2 w/2 h/2
        l-w/2 w/2 -w/2
        l-w/2 -w/2 h/2
        l-w/2 -w/2 -w/2
        l+w/2 w/2 h/2
        l+w/2 w/2 -w/2
        l+w/2 -w/2 h/2
        l+w/2 -w/2 -w/2];

    face_propbar_xpos = [  1  2  4 3
        5  6  8  7
        1  2  6  5
        1  3  7  5
        4  2  6  5
        4  3  7  8 ];


    % Plot fuselage
    surf(xbody,ybody,zbody,'FaceColor',maincolor,'FaceAlpha',alpha,'EdgeColor',maincolor,'EdgeAlpha',alpha)

    fill3(xbody(1,:),ybody(1,:),zbody(1,:),maincolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
    fill3(xbody(2,:),ybody(2,:),zbody(2,:),maincolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)

    % Plot bars
    for i = 0:3
        % Decide color to apply
        if i == 0
            coloring = mainbarcolor;
        else
            coloring = barcolor;
        end
        R90deg = [0 -1 0; 1 0 0; 0 0 1];

        % Draw bars and propellers
        vertices = (rot*R90deg^(i)*vert_bar_xpos'*esc)' + kron(ones(length(vert_bar_xpos),1),position);
        patch('Vertices',vertices,'Faces',face_bar_xpos,'FaceColor', coloring,'LineWidth',LW,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha);
        vertices = (rot*R90deg^(i)*vert_propbar_xpos'*esc)' + kron(ones(length(vert_propbar_xpos),1),position);
        patch('Vertices',vertices,'Faces',face_propbar_xpos,'FaceColor',coloring,'LineWidth',LW,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha);

        % Compute appropriate proppeler coordinates
        for j=1:2
            prop = (rot*R90deg^(i)*([xproptmp(j,:)' yproptmp(j,:)' zproptmp(j,:)']*esc)')' ...
                + kron(ones(length(xproptmp),1),position);

            xprop(j,:) = prop(:,1);
            yprop(j,:) = prop(:,2);
            zprop(j,:) = prop(:,3);
        end

        surf(xprop,yprop,zprop,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)

        fill3(xprop(1,:),yprop(1,:),zprop(1,:),propcolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
        fill3(xprop(2,:),yprop(2,:),zprop(2,:),propcolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
    end

    if auto
        grid on;
        axis equal;
        hold off;
    end

end
