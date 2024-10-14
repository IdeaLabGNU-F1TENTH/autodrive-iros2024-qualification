FROM autodriveecosystem/autodrive_f1tenth_api:2024-iros-practice

RUN apt-get update
RUN apt-get install -y tmux

RUN git clone https://github.com/IdeaLabGNU-F1TENTH/autodrive-iros2024-qualification.git
RUN mv autodrive-iros2024-qualification/gap_follow ./src/
RUN mv autodrive-iros2024-qualification/autodrive.sh .
RUN rm -rf autodrive-iros2024-qualification
RUN . /opt/ros/foxy/setup.sh && colcon build
CMD ["autodrive.sh"]
