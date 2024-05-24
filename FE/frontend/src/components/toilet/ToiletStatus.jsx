// ToiletStatus.js
import React, { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import styles from "./ToiletStatus.module.css";
import ToiletGrid from "./ToiletGrid";
import ToiletInfo from "../../layouts/ToiletInfo";

const ToiletStatus = ({ selectFloor, restroomData }) => {
  const [restroomMen, setRestroomMen] = useState();
  const [restroomWomen, setRestroomWomen] = useState();
  const navigate = useNavigate();

  useEffect(() => {
    if (restroomData !== undefined) {
      const men = restroomData.filter(
        (restroom) => restroom.floor == selectFloor && restroom.gender == 0
      );

      const women = restroomData.filter(
        (restroom) => restroom.floor == selectFloor && restroom.gender == 1
      );
      setRestroomMen([men[0]]);
      setRestroomWomen([women[0]]);
    }
  }, [selectFloor, restroomData]);

  // 홈으로
  const handleHomeClick = () => {
    navigate(`/`);
  };

  return (
    <div className={styles.statusContainer}>
      <div className={styles.header}>
        <img
          className={styles.title}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
          onClick={handleHomeClick}
        />
        <div className={styles.floor}>{selectFloor}F</div>
      </div>
      <div className={styles.content}>
        <div className={styles.itemBox}>
          <p className={styles.textItem}>Men</p>
          <div className={styles.gridItem}>
            {restroomMen && restroomMen[0] != undefined ? (
              <ToiletGrid restroom={restroomMen} />
            ) : (
              //화장실 없음 이미지
              <div> </div>
            )}
          </div>
        </div>
        <div className={styles.itemBox}>
          <p className={styles.textItem}>Women</p>
          <div className={styles.gridItem}>
            {restroomWomen && restroomWomen[0] != undefined ? (
              <ToiletGrid restroom={restroomWomen} />
            ) : (
              //화장실 없음 이미지
              <div> </div>
            )}
          </div>
        </div>
      </div>
      <ToiletInfo />
    </div>
  );
};

export default ToiletStatus;
