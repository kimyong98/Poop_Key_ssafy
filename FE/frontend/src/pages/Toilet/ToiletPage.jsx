// ToiletPage.js
import styles from "./ToiletPage.module.css";
import React, { useEffect, useState } from "react";
import { getRestRoom } from "../../util/API";
import { useLocation, useParams } from "react-router-dom";
import FloorBar from "../../components/toilet/FloorBar";
import ToiletStatus from "../../components/toilet/ToiletStatus";

const ToiletPage = () => {
  const { buildingId } = useParams(); // 경로에 있는 빌딩 아이디
  const [restroomData, setRestroomData] = useState();
  const [selectFloor, setSelectFloor] = useState(1);
  const [isUpdate, setIsUpdate] = useState(0);
  const location = useLocation(); // ToiletReport에서 navigate를 통해 전달 받은 파라미터

  // 전체 데이터
  // buildingId를 넘겨주어 전체 화장실 데이터를 받아온 후 저장.
  useEffect(() => {
    const getToiletData = async () => {
      try {
        const response = await getRestRoom(buildingId);
        setRestroomData(response);
      } catch (e) {
        console.log(e);
      }
    };
    getToiletData();
  }, [buildingId, isUpdate]);

  // ToiletReport에서 제출/취소를 했을 때 navigate를 통해 넘어온 층으로 다시 돌아간다.
  // 2층 화장실에서 신고 했을 때, 1층으로 돌아갔던 현상을 방지
  useEffect(() => {
    if (location.state != null) {
      setSelectFloor(Number(location.state.floor));
    }
  }, [location]);

  // FloorBar에 넘겨준 핸들러 함수로, FloorBar에서 층을 선택하면 ToiletPage에서 상태가 변경 됨
  // ToiletPage에서 변경 시켜야만 ToiletStatus도 랜더링 된다.
  const handleFloorClick = (floor) => {
    setSelectFloor(floor);
  };

  const updataWebSocket = (data) => {
    setIsUpdate(data);
  };

  return (
    <div className={styles.toiletContainer}>
      <ToiletStatus
        selectFloor={selectFloor}
        restroomData={restroomData}
        isUpdate={isUpdate}
      />
      <FloorBar
        selectFloor={selectFloor}
        restroomData={restroomData}
        handleFloorClick={handleFloorClick}
        updataWebSocket={updataWebSocket}
      />
    </div>
  );
};

export default ToiletPage;
