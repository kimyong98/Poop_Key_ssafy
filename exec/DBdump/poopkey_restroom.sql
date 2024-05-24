-- MySQL dump 10.13  Distrib 8.0.34, for Win64 (x86_64)
--
-- Host: 127.0.0.1    Database: poopkey
-- ------------------------------------------------------
-- Server version	8.0.35

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `restroom`
--

DROP TABLE IF EXISTS `restroom`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `restroom` (
  `restroom_id` int NOT NULL AUTO_INCREMENT,
  `building_id` int NOT NULL,
  `floor` int NOT NULL DEFAULT '1',
  `congestion` int NOT NULL DEFAULT '0' COMMENT '0:원활, 1:혼잡, 2:포화',
  `gender` int NOT NULL DEFAULT '0' COMMENT '0:남성, 1:여성',
  `cleaning` int NOT NULL DEFAULT '0' COMMENT '0:청소x, 1:청소중',
  `height` int NOT NULL DEFAULT '1' COMMENT '인덱스 번호 1부터 시작',
  `width` int NOT NULL DEFAULT '1' COMMENT '인덱스 번호 1부터 시작',
  `last_cleaning_time` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`restroom_id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `restroom`
--

LOCK TABLES `restroom` WRITE;
/*!40000 ALTER TABLE `restroom` DISABLE KEYS */;
INSERT INTO `restroom` VALUES (1,1,1,0,0,0,3,3,'2024-02-15 04:29:18'),(2,1,1,1,1,0,3,3,'2024-02-15 04:29:18'),(3,1,2,0,0,0,3,3,'2024-02-15 04:29:18'),(4,1,2,1,1,1,3,3,'2024-02-15 04:29:18');
/*!40000 ALTER TABLE `restroom` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2024-02-15 13:31:08
