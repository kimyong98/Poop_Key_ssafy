// ToiletPage.js
import styles from "./Home.module.css";
import React, { useEffect, useState, useRef } from "react";
import { getAllBuilding } from "../../util/API";
import { useNavigate } from "react-router-dom";
import LoginHeader from "../../layouts/LoginHeader";

const Home = () => {
  const [buildingData, setBuildingData] = useState();
  const navigate = useNavigate();

  useEffect(() => {
    const getBuildingData = async () => {
      try {
        const response = await getAllBuilding();
        console.log(response);
        setBuildingData(response);
      } catch (e) {
        console.log(e);
      }
    };
    getBuildingData();
  }, []);

  const handleBuildingClick = (buildingId, buildingName) => {
    navigate(`/toilet/${buildingId}?name=${buildingName}`);
  };

  return (
    <div className={styles.container}>
      <LoginHeader />
      <div>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
        />
        <div>
          <div>
            {buildingData &&
              buildingData.map((building) => (
                <div
                  className={styles.building}
                  key={building.buildingId}
                  onClick={() => {
                    handleBuildingClick(
                      building.buildingId,
                      building.buildingName
                    );
                  }}
                >
                  {building.buildingName} 역삼 멀티캠퍼스
                </div>
              ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default Home;
