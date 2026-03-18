import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Etsitään hoverboard_driver-paketin asennuskansion polku järjestelmästä.
    # Näin vältetään kovakoodatut tiedostopolut (esim. /home/ubuntu/...), 
    # ja koodi toimii riippumatta siitä, mihin kansioon työtila on luotu.
    pkg_share = FindPackageShare('hoverboard_driver')
    
    # Konfiguraatiotiedostojen dynaamiset polut.
    # Yhdistetään paketin polku ja config-kansion tiedostot, jotta nodet tietävät
    # mistä hakea aiemmin määritellyt YAML-parametrit.
    gps_config_path = PathJoinSubstitution([pkg_share, 'config', 'gps.yaml'])
    ekf_config_path = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])

    # --- Ublox DGNSS Node ---
    # Tämä solmu on suora rajapinta ardusimple-laitteeseesi (simpleRTK2B).
    # Se lukee USB:n (sarjaportin) kautta tulevaa NMEA/UBX-dataa ja julkaisee sen ROS 2:n 
    # ymmärtämässä muodossa (esim. /ublox_dgnss/fix -topicciin).
    ublox_node = Node(
        package='ublox_dgnss_node',
        executable='ublox_dgnss_node',
        name='ublox_dgnss',
        output='both',  # Tulostaa lokit sekä terminaaliin että ROS-lokeihin
        parameters=[gps_config_path],
        remappings=[
            # PERUSTELU: Jotta GPS saavuttaa senttimetritarkkuuden (RTK Fix), sen täytyy
            # saada korjausdataa (RTCM-viestejä) tukiasemalta. Ublox-ajuri kuuntelee oletuksena
            # topicia '/rtcm_in'. Me ohjaamme (remap) ntrip_clientin datan suoraan tähän topicciin.
            ('/rtcm_in', '/ntrip_client/rtcm')
        ]
    )

    # --- NTRIP Client Node ---
    # Tämä solmu yhdistää verkon yli (Wi-Fi/4G) RTKBase-tukiasemaasi.
    # Se lataa jatkuvaa RTCM-korjausvirtaa ja julkaisee sen ROS-järjestelmään.
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client_node',
        output='both',
        parameters=[gps_config_path],
        remappings=[
            # PERUSTELU: Ntrip-client julkaisee oletuksena dataa topicciin '/rtcm'. 
            # Nimämme sen uudelleen '/ntrip_client/rtcm', jotta se yhdistyy saumattomasti 
            # yllä olevaan Ublox-noden remappaukseen. Näin korjausdata virtaa internetistä -> Ubloxiin.
            ('/rtcm', '/ntrip_client/rtcm')
        ]
    )

    # --- LISÄTTY: Virallinen UBX -> NavSatFix muunnin ---
    # Tämä solmu lukee Ublox-noden lähettämää raakadataa (/ubx_nav_pvt, /ubx_nav_cov)
    # ja luo niistä automaattisesti standardin sensor_msgs/NavSatFix -viestin.
    navsatfix_transform_node = Node(
        package='ublox_nav_sat_fix_hp_node',
        executable='ublox_nav_sat_fix_hp',
        name='ublox_nav_sat_fix_hp',
        output='screen',
        remappings=[
            # LISÄTTY PERUSTELU: Solmu julkaisee oletuksena topicia '/fix'. 
            # Reititetään se nimelle '/ublox_dgnss/fix', jota alla oleva 
            # navsat_transform_node on jo ohjelmoitu kuuntelemaan.
            ('/fix', '/ublox_dgnss/fix') 
        ]
    )


    # --- NavSat Transform Node ---
    # GPS puhuu kieltä WGS84 (pituuspiiri, leveyspiiri, korkeus), jota on hankala käyttää
    # paikallisessa matematiikassa. Tämä solmu on kääntäjä: se ottaa GPS-koordinaatit
    # ja muuntaa ne tavallisiksi X/Y -metreiksi "map"-koordinaatistossa.
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            # PERUSTELU REMAPPIIN: 
            # 1. NavSat tarvitsee IMU-dataa tietääkseen mihin suuntaan maapalloa robotti katsoo.
            ('imu/data', '/imu'),

            # 2. Se tarvitsee Ubloxin julkaiseman tarkan GPS-paikan (Lat/Lon).
            ('gps/fix', '/ublox_dgnss/fix'), 

            # 3. Se tarvitsee robotin *globaalin* sijainnin EKF:ltä (odometry/global), jotta se osaa 
            # ankkuroida ensimmäisen GPS-pisteen oikein kartalle ja laskea muunnoksen.
            # Oletuksena se kuuntelisi topicia 'odometry/filtered', mutta koska meillä on kaksi
            # EKF:ää (paikallinen ja globaali), meidän on pakko spesifioida, että tämä käyttää globaalia.
            ('odometry/filtered', '/odometry/global')
        ]
    )

    # --- Globaali EKF Node ---
    # Tämä on järjestelmän "aivot" paikannuksen suhteen. Se kerää yhteen pyörien pyörimisen (odom0),
    # IMUn kallistukset ja suunnan (imu0) sekä NavSat-muuntimelta tulevat metriset GPS-koordinaatit (odom1).
    # Näistä se laskee optimaalisen, tarkan arvion robotin sijainnista ja julkaisee 'map -> odom' TF-muunnoksen.
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            # PERUSTELU: Oletuksena ekf_node julkaisee sijaintinsa topicciin '/odometry/filtered'.
            # Koska paikallinen EKF (joka on jo diffbot.launch.py -tiedostossa) käyttää luultavasti
            # tuota samaa oletusnimeä, nimeämme tämän tarkoituksella '/odometry/global'.
            # Näin ROS2-verkossa ei tule ristiriitoja kahden eri EKF:n välillä.
            ('odometry/filtered', '/odometry/global')
        ]
    )

    # Palautetaan LaunchDescription, joka kertoo ROS 2:lle, että kaikki nämä 
    # neljä solmua pitää käynnistää samanaikaisesti, kun tämä tiedosto ajetaan.
    return LaunchDescription([
        ublox_node,
        ntrip_node,
        navsatfix_transform_node,
        navsat_transform_node,
        ekf_global_node
    ])