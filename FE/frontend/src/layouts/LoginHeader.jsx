import styles from "./LoginHeader.module.css";
import { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { useNavigate } from "react-router-dom";
const LoginHeader = () => {
  const [isLogin, setIsLogin] = useState(false);
  const navigate = useNavigate();

  useEffect(() => {
    if (localStorage.length > 0) {
      setIsLogin(true);
    } else {
      setIsLogin(false);
    }
  }, []);

  const handleLogInClick = () => {
    navigate(`/login`);
  };

  const handleJoinClick = () => {
    navigate(`/regist`);
  };

  const handleLogoutClick = () => {
    localStorage.clear();
    setIsLogin(false);
  };

  return (
    <div className={styles.container}>
      {/* {!isLogin ? (
        <div className={styles.login}>
          <div onClick={handleLogInClick}>로그인</div>
          <div onClick={handleJoinClick}>회원가입</div>
        </div>
      ) : (
        <div className={styles.logout}>
          <div onClick={handleLogoutClick}>로그아웃</div>
        </div>
      )} */}
    </div>
  );
};
export default LoginHeader;
