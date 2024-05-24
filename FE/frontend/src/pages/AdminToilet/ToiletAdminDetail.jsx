import styles from "./ToiletAdminDetail.module.css";
import StateChange from "../../components/adminToilet/StateChange";

const ToiletAdminDetatil = () => {
  return (
    <>
      <StateChange />
    </>
  );
};
export default ToiletAdminDetatil;

//select/ option -> useParams로 경로 데이터 받아서, 신고 요청 + 통계데이터 받아와서 일부 띄워주기 -> 통계데이터는 화장실 id로 호출
