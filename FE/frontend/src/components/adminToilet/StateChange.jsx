import styles from "./StateChange.module.css";
import { useNavigate, useParams } from "react-router-dom";
import { useEffect, useState } from "react";
import { updateStatus, getMailLocation } from "../../util/AdminAPI"; // 점검 변경하는 axios
import StateItem from "./StateItem";

const stateList = [
  {
    id: 0,
    state_img: process.env.PUBLIC_URL + `/assets/toiletcolor.png`,
    state_descript: "사용가능",
  },
  {
    id: 2,
    state_img: process.env.PUBLIC_URL + `/assets/warning.png`,
    state_descript: "고장",
  },
  {
    id: 3,
    state_img: process.env.PUBLIC_URL + `/assets/repaircolor.png`,
    state_descript: "점검중",
  },
];

const StateChange = () => {
  const [state, setState] = useState(0);
  const [stallLocation, setStallLocation] = useState();
  const { buildingId, stallId, floor } = useParams();
  const navigate = useNavigate();

  useEffect(() => {
    const stallLocation = async (stallId) => {
      try {
        const response = await getMailLocation(stallId);
        const floor = `${response.floor}층 `;
        const gender = response.gender == 0 ? "남자화장실 " : "여자화장실 ";
        const location = `${findLocation(response.list, stallId)}번 칸 `;

        setStallLocation({
          floor: floor,
          gender: gender,
          location: location,
        });
      } catch (e) {
        console.log(e);
      }
    };
    stallLocation(stallId);
  }, []);

  // 화장실 칸 번호를 리턴하는 함수
  const findLocation = (list, stallId) => {
    let blank = 0;
    for (let i = 0; i < list.length; i++) {
      if (list[i].content == 0) blank++;
      if (list[i].stallId == stallId) {
        return i + 1 - blank;
      }
    }
  };

  const handleClickEmotion = (state) => {
    setState(state);
  };

  const handleSubmit = async () => {
    const data = {
      stallId: parseInt(stallId), // 화장실칸ID
      state: state, // 변경 내용 (사용가능:0 , 고장:2, 점검중:3)
    };
    await updateStatus(data);

    navigate(`/admin/toilet/${buildingId}`, { state: { floor: floor } });
  };

  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
        />
        <div className={styles.text}>Admin State Change</div>
      </div>
      <div className={styles.stateContainer}>
        <section>
          <div className={styles.stateType}>상태변경</div>
          <div className={styles.state_list_wrapper}>
            {stateList.map((it) => (
              <StateItem
                key={it.id}
                {...it}
                onClick={handleClickEmotion}
                //isSelected -> 선택된 아이템만 true -> style을 위한 변수
                isSelected={it.id === state}
              />
            ))}
          </div>
        </section>
        <section>
          <div>
            <div className={styles.state_content}>
              <img
                className={styles.contentImg}
                src={process.env.PUBLIC_URL + `/assets/location.png`}
              />
              <span
                className={styles.contentText}
                onClick={() =>
                  navigate(`/admin/toilet/${buildingId}`, {
                    state: { floor: floor },
                  })
                }
              >
                {stallLocation &&
                  stallLocation.floor +
                    stallLocation.gender +
                    stallLocation.location}
              </span>
            </div>
          </div>
        </section>
        <section className={styles.buttonContainer}>
          <div className={styles.button} onClick={handleSubmit}>
            제출하기{" "}
          </div>
          <div className={styles.button} onClick={() => navigate(`/admin/1`)}>
            취소하기
          </div>
        </section>
      </div>
    </div>
  );
};

export default StateChange;
