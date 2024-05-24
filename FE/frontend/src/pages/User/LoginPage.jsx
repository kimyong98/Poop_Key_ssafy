import styles from "./LoginPage.module.css";
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { userLogIn } from "../../util/API";

const LoginPage = () => {
  const navigate = useNavigate();
  const [loginData, setLoginData] = useState({
    email: "",
    password: "",
  });

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setLoginData({
      ...loginData,
      [name]: value,
    });
  };

  const handleLoginSubmit = async (e) => {
    e.preventDefault();
    try {
      const data = {
        email: loginData.email,
        password: loginData.password,
      };
      const response = await userLogIn(data);
      console.log("Login successful", response);

      // 로컬 스토리지에 저장
      // 현재는 보냈던 데이터를 다시 쓰고 있지만 서버에서 email, 이름, 서버 보내주면 좋음
      localStorage.removeItem("UserToken");
      localStorage.removeItem("user");
      localStorage.setItem("UserToken", response.token);
      localStorage.setItem("user", 1);

      navigate("/");
    } catch (error) {
      console.log("Login failed", error);
    }
  };
  return (
    <div className={styles.container}>
      <img
        className={styles.logo}
        src={process.env.PUBLIC_URL + `/assets/Logo.png`}
      />
      <form onSubmit={handleLoginSubmit}>
        <div>
          <input
            type="text"
            name="email"
            placeholder="Email"
            value={loginData.email}
            onChange={handleInputChange}
          />
        </div>
        <div>
          <input
            type="password"
            placeholder="Password"
            name="password"
            value={loginData.password}
            onChange={handleInputChange}
          />
        </div>
        <div className={styles.button}>
          <button type="submit">Login</button>
        </div>
      </form>
    </div>
  );
};
export default LoginPage;
