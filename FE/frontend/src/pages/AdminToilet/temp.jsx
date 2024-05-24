import styles from "./ToiletStatistics.module.css";
import { getStatistics } from "../../util/AdminAPI";
import { getAllBuilding, getRestRoom } from "../../util/API";
import { useEffect, useState } from "react";
import { useParams, useNavigate } from "react-router-dom";

const ToiletStatistics = () => {
  const { buildingId } = useParams();
  const [gender, setGender] = useState(0);
  const [floor, setFloor] = useState(1);
  const [floorArr, setFloorArr] = useState();
  const [restroomData, setRestroomData] = useState();
  const [buildingData, setBuildingData] = useState();
  const [buildingName, setBuildingName] = useState();
  const [statisticData, setStatisticData] = useState();
  const navigate = useNavigate();

  // 빌딩 아이디는 초기 접속시 받아올 것, 층에 대한 데이터 받아올 것, 층수, 성별만 사용자가 선택
  // 해당층에 대한 정보가 없거나 데이터가 없으면 string이나옴
  useEffect(() => {
    const getToiletData = async () => {
      try {
        const resRestroom = await getRestRoom(buildingId);
        const resBuilding = await getAllBuilding();
        setRestroomData(resRestroom);
        setBuildingData(resBuilding);
      } catch (e) {
        console.log(e);
      }
    };
    getToiletData();
  }, [buildingId]);

  // 빌딩 이름 추출
  useEffect(() => {
    setBuildingName(
      buildingData &&
        buildingData.filter((data) => data.buildingId == buildingId)[0]
          .buildingName
    );
  }, [buildingData]);

  // 화장실 데이터를 통해 현재 존재하는 층수를 얻는다. (중복제거, 오름차순)
  useEffect(() => {
    const uniqueFloorArr = Array.from(
      new Set(restroomData && restroomData.map((data) => data.floor))
    ).sort(function (a, b) {
      return a - b;
    });

    if (uniqueFloorArr != undefined) {
      setFloorArr([...uniqueFloorArr]);
    }
  }, [restroomData]);

  // 초기 통계 데이터를 얻는다.
  useEffect(() => {
    const func = async () => {
      try {
        const response = await getStatistics("SSAFY 서울 캠퍼스", "1", "0");
        setStatisticData(response);
      } catch (e) {
        console.log(e);
      }
    };
    func();
  }, []);

  const handleSearchClick = async () => {
    try {
      const response = await getStatistics(buildingName, floor, gender);
      setStatisticData(response);
    } catch (e) {
      console.log(e);
    }
  };

  const handleFloorChange = (e) => {
    const { value } = e.target;
    setFloor(value);
  };

  const handleGenderChange = (e) => {
    const { value } = e.target;
    setGender(value);
    console.log(value, "성별");
  };

  // 홈으로
  const handleHomeClick = () => {
    navigate(`/admin/1`);
  };

  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
          onClick={handleHomeClick}
        />
        <div className={styles.text}>Admin Statistics</div>
      </div>
      <div className={styles.stContainer}>
        <div className={styles.selectOption}>
          <div className={styles.sizeOption}>
            <select
              className={styles.sizeOptionSelect}
              onChange={handleFloorChange}
            >
              {floorArr &&
                floorArr.map((floor) => (
                  <option key={floor} value={floor}>
                    {" "}
                    {floor}F
                  </option>
                ))}
            </select>
          </div>
          <div className={styles.sizeOption}>
            <select
              className={styles.sizeOptionSelect}
              onChange={handleGenderChange}
            >
              <option value={0}>남자</option>
              <option value={1}>여자</option>
            </select>
          </div>
          <div className={styles.sizeOption}>
            <div
              className={styles.sizeOptionSubmit}
              onClick={handleSearchClick}
            >
              제출
            </div>
          </div>
        </div>

        {setStatisticData && (
          <div className={styles.table}>
            <table>
              <thead>
                <tr>
                  <th>No</th>
                  <th>사용횟수</th>
                  <th>최근청소</th>
                  <th>휴지교체</th>
                  <th>고장횟수</th>
                </tr>
              </thead>
              <tbody>
                {statisticData &&
                  statisticData.list.map((data, index) => {
                    const date = data.lastCleanDate.split("T")[0];

                    return (
                      <tr key={index}>
                        <td>{index + 1}</td>
                        <td>{data.usedNumber}</td>
                        <td>{date}</td>
                        <td>{data.tissueChangeNumber}</td>
                        <td>{data.breakNumber}</td>
                      </tr>
                    );
                  })}
              </tbody>
            </table>
          </div>
        )}
      </div>
    </div>
  );
};

export default ToiletStatistics;
