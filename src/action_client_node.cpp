#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment2_rt/PositionVelocity.h>
#include <thread>
#include <mutex>

// Publisher per la posizione e velocità
ros::Publisher position_velocity_pub;

// Messaggio personalizzato
assignment2_rt::PositionVelocity pos_vel_msg;

// Action client globale per accedere dallo stato e dall'input dell'utente
std::shared_ptr<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>> ac_ptr;

// Mutex per sincronizzare l'accesso tra thread
std::mutex ac_mutex;

// Variabile per monitorare se il goal è completato
std::atomic<bool> goal_reached(false); 

// Callback per i dati di odometria
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Estrarre posizione e velocità dall'odometria
    pos_vel_msg.x = msg->pose.pose.position.x;
    pos_vel_msg.y = msg->pose.pose.position.y;
    pos_vel_msg.vel_x = msg->twist.twist.linear.x;
    pos_vel_msg.vel_z = msg->twist.twist.angular.z;

    // Pubblicare il messaggio personalizzato
    position_velocity_pub.publish(pos_vel_msg);
}

// Thread per monitorare lo stato del goal
void monitorGoalStatus() {
    ros::Rate rate(10); // Frequenza del monitoraggio
    actionlib::SimpleClientGoalState::StateEnum last_state = actionlib::SimpleClientGoalState::LOST;
    while (ros::ok()) {
        ac_mutex.lock();
        if (ac_ptr) {
            actionlib::SimpleClientGoalState state = ac_ptr->getState();
            if (state.state_ != last_state) { // Controlla se lo stato è cambiato
                last_state = state.state_; // Aggiorna lo stato precedente
                if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Target raggiunto!");
                    goal_reached = true; // Segnala che il goal è stato raggiunto
                } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                    ROS_INFO("Movimento in corso verso il target...");
                } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
                    ROS_INFO("Il target è stato annullato.");
                    goal_reached = true; // Segnala che il goal è stato annullato
                } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                    ROS_WARN("Il goal è fallito.");
                    goal_reached = true; // Segnala che il goal è fallito
                } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                    ROS_INFO("Il goal è in attesa di esecuzione...");
                }
            }
        }
        ac_mutex.unlock();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // Inizializzare il publisher
    position_velocity_pub = nh.advertise<assignment2_rt::PositionVelocity>("position_velocity", 10);

    // Subscriber al topic /odom
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Action client
    ac_ptr = std::make_shared<actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>>("/reaching_goal", true);

    // Attendere che il server sia disponibile
    ROS_INFO("In attesa del server dell'Action...");
    ac_ptr->waitForServer();
    ROS_INFO("Server dell'Action disponibile.");

    // Avviare il thread per monitorare lo stato del goal
    std::thread monitor_thread(monitorGoalStatus);

    while (ros::ok()) {
        // Menu per l'utente
        std::string command;
        ROS_INFO("Digita 'set' per impostare un goal o 'cancel' per annullare:");
        std::cin >> command;

        if (command == "set") {
            // Lettura delle coordinate del target
            double x, y;
            ROS_INFO("Inserisci le coordinate del target (x y):");

            // Controllo input dell'utente
            if (!(std::cin >> x >> y)) {
                ROS_ERROR("Input non valido. Inserire due numeri (x e y).");
                std::cin.clear(); // Ripristina lo stato dello stream
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignora input errati
                continue;
            }

            // Impostare e inviare il goal
            assignment_2_2024::PlanningGoal goal;
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;

            ac_mutex.lock();
            ac_ptr->sendGoal(goal);
            ac_mutex.unlock();

            goal_reached = false; // Resetta lo stato
            ROS_INFO("Goal inviato a (%.2f, %.2f).", x, y);

        } else if (command == "cancel") {
            // Annulla il goal corrente
            ac_mutex.lock();
            ac_ptr->cancelGoal();
            ac_mutex.unlock();
            ROS_INFO("Goal annullato.");
        } else {
            ROS_WARN("Comando non riconosciuto. Digita 'set' o 'cancel'.");
            continue;
        }

        // Attendi fino a quando il goal non è completato
        while (!goal_reached && ros::ok()) {
            ros::Duration(0.1).sleep(); // Dormi brevemente per ridurre il carico della CPU
        }

        ros::spinOnce();
    }

    // Unire il thread prima di uscire
    monitor_thread.join();

    return 0;
}

