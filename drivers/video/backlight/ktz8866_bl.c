#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/pwm.h>
#include <linux/leds.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/bitops.h>
#include <ktz8866_bl.h>

//#define ktz_DEBUG

//#define ktz8866_PINCTRL_DEFAULT "ktz8866_default"
//#define ktz8866_PINCTRL_SLEEP  "ktz8866_sleep"

static struct ktz8866_chip *ktz8866_global = NULL;
static int last_brightness = 0;
static int bl_force_flag = 0;

static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);
#define LOG_DBG(fmt, args...)  do { if (debug_mask) { printk(KERN_ERR "[%s]:: " fmt, __func__, ## args); } } while (0)

#if 0
int ktz8866_brightness_table_reg4[256] = {0x01,0x02,0x04,0x04,0x07,0x02,0x00,0x06,0x04,0x02,0x03,0x04,0x05,0x06,0x02,0x06,0x02,
			0x06,0x02,0x06,0x02,0x06,0x02,0x04,0x05,0x06,0x05,0x03,0x00,0x05,0x02,0x06,0x02,0x06,0x02,0x06,0x02,0x06,0x02,0x06,
			0x02,0x06,0x02,0x06,0x02,0x06,0x01,0x04,0x07,0x02,0x05,0x00,0x03,0x06,0x01,0x04,0x07,0x02,0x05,0x00,0x03,0x05,0x07,
			0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x00,0x01,0x02,0x03,
			0x04,0x05,0x06,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x06,0x05,
			0x04,0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x07,0x05,0x03,0x01,0x07,0x05,0x03,0x01,0x07,0x05,0x03,
			0x01,0x07,0x05,0x03,0x00,0x05,0x02,0x07,0x04,0x01,0x06,0x03,0x00,0x05,0x02,0x07,0x04,0x01,0x06,0x03,0x00,0x05,0x02,
			0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,0x03,0x07,
			0x03,0x07,0x03,0x07,0x03,0x06,0x01,0x04,0x07,0x02,0x05,0x00,0x03,0x06,0x01,0x04,0x07,0x02,0x05,0x00,0x03,0x06,0x01,
			0x04,0x07,0x02,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,
			0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,0x03,0x05,0x07,0x01,
			0x03,0x05,0x07,0x01,0x03,0x04,0x05,0x06,0x07};
int ktz8866_brightness_table_reg5[256] = {0x00,0x06,0x0C,0x11,0x15,0x1A,0x1E,0x21,0x25,0x29,0x2C,0x2F,0x32,0x35,0x38,0x3A,0x3D,
			0x3F,0x42,0x44,0x47,0x49,0x4C,0x4E,0x50,0x52,0x54,0x56,0x58,0x59,0x5B,0x5C,0x5E,0x5F,0x61,0x62,0x64,0x65,0x67,0x68,
			0x6A,0x6B,0x6D,0x6E,0x70,0x71,0x73,0x74,0x75,0x77,0x78,0x7A,0x7B,0x7C,0x7E,0x7F,0x80,0x82,0x83,0x85,0x86,0x87,0x88,
			0x8A,0x8B,0x8C,0x8D,0x8F,0x90,0x91,0x92,0x94,0x95,0x96,0x97,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F,0xA1,0xA2,0xA3,0xA4,
			0xA5,0xA6,0xA7,0xA8,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xB9,0xBA,0xBB,
			0xBC,0xBD,0xBE,0xBF,0xC0,0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC6,0xC7,0xC8,0xC9,0xC9,0xCA,0xCB,0xCC,0xCC,0xCD,0xCE,
			0xCF,0xCF,0xD0,0xD1,0xD2,0xD2,0xD3,0xD3,0xD4,0xD5,0xD5,0xD6,0xD7,0xD7,0xD8,0xD8,0xD9,0xDA,0xDA,0xDB,0xDC,0xDC,0xDD,
			0xDD,0xDE,0xDE,0xDF,0xDF,0xE0,0xE0,0xE1,0xE1,0xE2,0xE2,0xE3,0xE3,0xE4,0xE4,0xE5,0xE5,0xE6,0xE6,0xE7,0xE7,0xE8,0xE8,
			0xE9,0xE9,0xEA,0xEA,0xEB,0xEB,0xEC,0xEC,0xEC,0xED,0xED,0xEE,0xEE,0xEE,0xEF,0xEF,0xEF,0xF0,0xF0,0xF1,0xF1,0xF1,0xF2,
			0xF2,0xF2,0xF3,0xF3,0xF3,0xF4,0xF4,0xF4,0xF4,0xF5,0xF5,0xF5,0xF5,0xF6,0xF6,0xF6,0xF6,0xF7,0xF7,0xF7,0xF7,0xF8,0xF8,
			0xF8,0xF8,0xF9,0xF9,0xF9,0xF9,0xFA,0xFA,0xFA,0xFA,0xFB,0xFB,0xFB,0xFB,0xFC,0xFC,0xFC,0xFC,0xFD,0xFD,0xFD,0xFD,0xFE,
			0xFE,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#endif
int ktz8866_brightness_table_optimaztion[2048] = {
			0,1,8,14,20,26,32,38,44,50,57,64,70,76,82,88,94,100,105,110,115,120,125,130,135,140,145,150,155,159,163,167,171,175,180,
			185,190,194,198,202,206,210,214,218,222,226,230,234,237,240,244,248,252,256,260,264,267,270,274,278,282,286,290,294,397,
			300,304,308,312,316,320,324,337,330,334,337,340,343,346,349,352,355,359,362,365,368,371,374,377,380,384,387,390,393,396,
			399,402,405,409,412,415,418,421,424,427,430,433,436,439,442,444,446,448,450,453,456,459,462,464,466,468,470,473,476,479,
			482,484,486,488,490,493,496,499,502,504,506,508,510,513,516,519,522,524,526,528,530,533,536,539,542,544,546,548,550,553,
			556,559,562,564,566,568,570,573,576,579,582,584,586,588,590,593,596,599,602,604,606,608,610,613,616,618,620,622,624,626,
			628,631,633,635,637,639,641,643,645,648,650,652,654,656,658,660,662,664,666,668,670,672,674,676,677,679,681,683,685,687,
			689,690,691,693,695,697,699,701,702,703,704,706,708,710,712,714,715,716,717,719,721,723,725,727,728,729,730,732,734,736,
			738,739,740,741,742,744,746,748,749,750,751,752,754,756,757,758,759,760,762,764,766,768,769,770,771,772,774,776,778,780,
			781,782,783,784,786,788,790,792,793,794,795,796,798,800,802,804,805,806,807,808,810,812,814,816,817,818,819,820,822,824,
			826,828,829,830,831,832,834,836,838,840,841,842,843,844,846,848,850,852,853,854,855,856,858,860,862,864,865,866,867,868,
			870,872,874,876,877,878,879,880,882,884,886,888,889,890,891,892,894,896,898,900,901,902,903,904,906,908,910,912,913,914,
			915,916,918,920,921,923,924,925,926,927,929,931,932,934,935,936,937,938,940,942,943,945,946,947,948,949,951,953,954,956,
			957,958,959,960,962,964,965,967,968,969,970,971,973,975,976,978,979,980,981,982,984,986,987,989,990,991,992,993,995,997,
			998,1000,1001,1002,1003,1004,1006,1008,1009,1011,1012,1013,1014,1015,1017,1019,1020,1022,1023,1024,1025,1026,1028,1030,
			1031,1033,1034,1035,1036,1037,1039,1041,1042,1044,1045,1046,1047,1048,1050,1052,1053,1055,1056,1057,1058,1059,1061,1063,
			1064,1066,1067,1068,1069,1070,1072,1074,1075,1077,1078,1079,1080,1081,1083,1084,1085,1087,1088,1089,1090,1091,1093,1094,
			1095,1097,1098,1099,1100,1101,1103,1104,1105,1107,1108,1109,1110,1111,1113,1114,1115,1117,1118,1119,1120,1121,1123,1124,
			1125,1127,1128,1129,1130,1131,1133,1134,1135,1137,1138,1139,1140,1141,1143,1144,1145,1147,1148,1149,1150,1151,1153,1154,
			1155,1157,1158,1159,1160,1161,1163,1164,1165,1167,1168,1169,1170,1171,1173,1174,1175,1177,1178,1179,1180,1181,1183,1184,
			1185,1187,1188,1189,1190,1191,1193,1194,1195,1197,1198,1199,1200,1201,1203,1204,1205,1207,1208,1209,1210,1211,1213,1214,
			1215,1217,1218,1219,1220,1221,1223,1224,1225,1227,1228,1229,1230,1231,1232,1233,1234,1236,1237,1238,1239,1240,1241,1242,
			1243,1245,1246,1247,1248,1249,1250,1251,1252,1254,1255,1256,1257,1258,1259,1260,1261,1263,1264,1265,1266,1267,1268,1269,
			1270,1272,1273,1274,1275,1276,1277,1278,1279,1281,1282,1283,1284,1285,1286,1287,1288,1290,1291,1292,1293,1294,1295,1296,
			1297,1299,1300,1301,1302,1303,1304,1305,1306,1308,1309,1310,1311,1312,1313,1314,1315,1317,1318,1319,1320,1321,1322,1323,
			1324,1326,1327,1328,1329,1330,1331,1332,1333,1335,1336,1337,1338,1339,1340,1341,1342,1344,1345,1346,1347,1348,1349,1350,
			1351,1353,1354,1355,1356,1357,1358,1359,1360,1362,1363,1364,1365,1366,1367,1368,1368,1369,1370,1371,1372,1373,1374,1375,
			1376,1377,1378,1379,1380,1381,1382,1383,1384,1385,1386,1387,1388,1389,1390,1391,1392,1393,1394,1395,1396,1397,1398,1399,
			1400,1401,1402,1403,1404,1405,1406,1407,1408,1409,1410,1411,1412,1413,1414,1415,1416,1417,1418,1419,1420,1421,1422,1423,
			1424,1425,1426,1427,1428,1429,1430,1431,1432,1433,1434,1435,1436,1437,1438,1439,1440,1441,1442,1443,1444,1445,1446,1447,
			1448,1449,1450,1451,1452,1453,1454,1455,1456,1457,1458,1459,1460,1461,1462,1463,1464,1465,1466,1467,1468,1469,1470,1471,
			1472,1473,1474,1475,1476,1477,1478,1479,1480,1481,1482,1483,1484,1485,1486,1487,1487,1488,1489,1490,1491,1492,1493,1494,
			1494,1495,1496,1497,1498,1499,1500,1501,1501,1502,1503,1504,1505,1506,1507,1508,1508,1509,1510,1511,1512,1513,1514,1515,
			1515,1516,1517,1518,1519,1520,1521,1522,1522,1523,1524,1525,1526,1527,1528,1529,1529,1530,1531,1532,1533,1534,1535,1536,
			1536,1537,1538,1539,1540,1541,1542,1543,1543,1544,1545,1546,1547,1548,1549,1550,1550,1551,1552,1553,1554,1555,1556,1557,
			1557,1558,1559,1560,1561,1562,1563,1564,1564,1565,1566,1567,1568,1569,1570,1571,1571,1572,1573,1574,1575,1576,1577,1578,
			1578,1579,1580,1581,1582,1583,1584,1585,1585,1586,1587,1588,1589,1589,1590,1591,1591,1592,1593,1594,1595,1595,1596,1597,
			1597,1598,1599,1600,1601,1601,1602,1603,1603,1604,1605,1606,1607,1607,1608,1609,1609,1610,1611,1612,1613,1613,1614,1615,
			1615,1616,1617,1618,1619,1619,1620,1621,1621,1622,1623,1624,1625,1625,1626,1627,1627,1628,1629,1630,1631,1631,1632,1633,
			1633,1634,1635,1636,1637,1637,1638,1639,1639,1640,1641,1642,1643,1643,1644,1645,1645,1646,1647,1648,1649,1649,1650,1651,
			1651,1652,1653,1654,1655,1655,1656,1657,1657,1658,1659,1660,1661,1661,1662,1663,1663,1664,1665,1666,1667,1667,1668,1669,
			1669,1670,1671,1672,1673,1673,1674,1675,1675,1676,1676,1677,1678,1678,1679,1680,1680,1681,1681,1682,1683,1683,1684,1685,
			1685,1686,1686,1687,1688,1688,1689,1690,1690,1691,1691,1692,1693,1693,1694,1695,1695,1696,1696,1697,1698,1698,1699,1700,
			1700,1701,1701,1702,1703,1703,1704,1705,1705,1706,1706,1707,1708,1708,1709,1710,1710,1711,1711,1712,1713,1713,1714,1715,
			1715,1716,1716,1717,1718,1718,1719,1720,1720,1721,1721,1722,1723,1723,1724,1725,1725,1726,1726,1727,1728,1728,1729,1730,
			1730,1731,1731,1732,1733,1733,1734,1735,1735,1736,1736,1737,1738,1738,1739,1740,1740,1741,1741,1742,1743,1743,1744,1745,
			1745,1746,1746,1747,1748,1748,1749,1750,1750,1751,1751,1752,1753,1753,1754,1755,1755,1756,1756,1757,1758,1758,1759,1760,
			1760,1761,1761,1762,1763,1763,1764,1765,1765,1766,1766,1767,1768,1768,1769,1770,1770,1771,1771,1772,1773,1773,1774,1775,
			1775,1776,1776,1776,1777,1777,1778,1779,1779,1780,1780,1780,1781,1781,1782,1783,1783,1784,1784,1784,1785,1785,1786,1787,
			1787,1788,1788,1788,1789,1789,1790,1791,1791,1792,1792,1792,1793,1793,1794,1795,1795,1796,1796,1796,1797,1797,1798,1799,
			1799,1800,1800,1800,1801,1801,1802,1803,1803,1804,1804,1804,1805,1805,1806,1807,1807,1808,1808,1808,1809,1809,1810,1811,
			1811,1812,1812,1812,1813,1813,1814,1815,1815,1816,1816,1816,1817,1817,1818,1819,1819,1820,1820,1820,1821,1821,1822,1823,
			1823,1824,1824,1824,1825,1825,1826,1827,1827,1828,1828,1828,1829,1829,1830,1831,1831,1832,1832,1832,1833,1833,1834,1835,
			1835,1836,1836,1836,1837,1837,1838,1839,1839,1840,1840,1840,1841,1841,1842,1843,1843,1844,1844,1844,1845,1845,1846,1847,
			1847,1848,1848,1848,1849,1849,1850,1851,1851,1852,1852,1852,1853,1853,1854,1855,1855,1856,1856,1856,1857,1857,1858,1859,
			1859,1860,1860,1860,1861,1861,1862,1863,1863,1864,1864,1864,1865,1865,1866,1867,1867,1868,1868,1868,1869,1869,1870,1871,
			1871,1872,1872,1872,1873,1873,1874,1875,1875,1876,1876,1876,1877,1877,1878,1879,1879,1880,1880,1880,1881,1881,1882,1883,
			1883,1884,1884,1884,1885,1885,1886,1886,1886,1887,1887,1887,1888,1888,1889,1889,1889,1890,1890,1890,1891,1891,1892,1892,
			1892,1893,1893,1893,1894,1894,1895,1895,1895,1896,1896,1896,1897,1897,1898,1898,1898,1899,1899,1899,1900,1900,1901,1901,
			1901,1902,1902,1902,1903,1903,1904,1904,1904,1905,1905,1905,1906,1906,1907,1907,1907,1908,1908,1908,1909,1909,1910,1910,
			1910,1911,1911,1911,1912,1912,1913,1913,1913,1914,1914,1914,1915,1915,1916,1916,1916,1917,1917,1917,1918,1918,1919,1919,
			1919,1920,1920,1920,1921,1921,1922,1922,1922,1923,1923,1923,1924,1924,1925,1925,1925,1926,1926,1926,1927,1927,1928,1928,
			1928,1929,1929,1929,1930,1930,1931,1931,1931,1932,1932,1932,1933,1933,1934,1934,1934,1935,1935,1935,1936,1936,1937,1937,
			1937,1938,1938,1938,1939,1939,1940,1940,1940,1941,1941,1941,1942,1942,1943,1943,1943,1944,1944,1944,1945,1945,1946,1946,
			1946,1947,1947,1947,1948,1948,1949,1949,1949,1950,1950,1950,1951,1951,1951,1951,1951,1952,1952,1952,1953,1953,1953,1953,
			1953,1954,1954,1954,1955,1955,1955,1955,1955,1956,1956,1956,1957,1957,1957,1957,1957,1958,1958,1958,1959,1959,1959,1959,
			1959,1960,1960,1960,1961,1961,1961,1961,1961,1962,1962,1962,1963,1963,1963,1963,1963,1964,1964,1964,1965,1965,1965,1965,
			1965,1966,1966,1966,1967,1967,1967,1967,1967,1968,1968,1968,1969,1969,1969,1969,1969,1970,1970,1970,1971,1971,1971,1971,
			1971,1972,1972,1972,1973,1973,1973,1973,1973,1974,1974,1974,1975,1975,1975,1975,1975,1976,1976,1976,1977,1977,1977,1977,
			1977,1978,1978,1978,1979,1979,1979,1979,1979,1980,1980,1980,1981,1981,1981,1981,1981,1982,1982,1982,1983,1983,1983,1983,
			1983,1984,1984,1984,1985,1985,1985,1985,1985,1986,1986,1986,1987,1987,1987,1987,1987,1988,1988,1988,1989,1989,1989,1989,
			1989,1990,1990,1990,1991,1991,1991,1991,1991,1992,1992,1992,1993,1993,1993,1993,1993,1994,1994,1994,1995,1995,1995,1995,
			1995,1996,1996,1996,1997,1997,1997,1997,1997,1998,1998,1998,1999,1999,1999,1999,1999,2000,2000,2000,2001,2001,2001,2001,
			2001,2002,2002,2002,2003,2003,2003,2003,2003,2004,2004,2004,2005,2005,2005,2005,2005,2006,2006,2006,2007,2007,2007,2007,
			2007,2008,2008,2008,2009,2009,2009,2009,2009,2010,2010,2010,2011,2011,2011,2011,2011,2012,2012,2012,2013,2013,2013,2013,
			2013,2014,2014,2014,2015,2015,2015,2015,2015,2016,2016,2016,2017,2017,2017,2017,2017,2018,2018,2018,2019,2019,2019,2019,
			2019,2020,2020,2020,2021,2021,2021,2021,2021,2022,2022,2022,2023,2023,2023,2023,2023,2024,2024,2024,2025,2025,2025,2025,
			2025,2026,2026,2026,2027,2027,2027,2027,2027,2028,2028,2028,2029,2029,2029,2029,2029,2030,2030,2030,2031,2031,2031,2031,
			2031,2032,2032,2032,2033,2033,2033,2033,2033,2034,2034,2034,2035,2035,2035,2035,2035,2036,2036,2036,2037,2037,2037,2037,
			2037,2038,2038,2038,2039,2039,2039,2039,2039,2040,2040,2040,2041,2041,2041,2041,2041,2042,2042,2042,2043,2043,2043,2043,
			2043,2044,2044,2044,2045,2045,2045,2045,2044,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,2045,
			2046,2046,2046,2046,2046,2046,2046,2046,2047,2047,2047,2047,2047,2047,2047};


static int ktz8866_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);

	if(ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int ktz8866_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg);

	if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
	}else{
		*val = ret;
	}

	LOG_DBG("Reading 0x%02x=0x%02x\n", reg, *val);

	return ret;
}

static int ktz8866_masked_write(struct i2c_client *client, int reg, u8 mask, u8 val)
{
	int rc;
	u8 temp;

	rc = ktz8866_read_reg(client, reg, &temp);
	if(rc < 0){
		dev_err(&client->dev, "failed to read reg=0x%x, rc=%d\n", reg, rc);
	}else{
		temp &= ~mask;
		temp |= val & mask;
		rc = ktz8866_write_reg(client, reg, temp);
		if(rc<0){
			dev_err(&client->dev, 
							"failed to write masked data. reg=%03x, rc=%d\n", reg, rc);
		}
	}

	//ktz8866_read_reg(client, reg, &temp);
	return rc;
}

static int ktz_find_bit(int x)
{
	int i = 0;

	while ((x = x >> 1))
		i++;

	return i+1;
}

void ktz_parse_dt(struct device *dev, struct ktz8866_chip *chip)
{
	struct device_node *np = dev->of_node;
	struct ktz8866_bl_pdata *pdata = chip->pdata;
	int rc=0;
	u32 bl_channel, temp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if(!pdata){
		dev_err(dev,"failed to alloc pdata in parse dt!\n");
		return;// -ENOMEM;
	}

	pdata->hwen_gpio = of_get_named_gpio(np, "hwen-gpio", 0);
	LOG_DBG("hwen --<%d>\n", pdata->hwen_gpio);

	pdata->pwm_mode = of_property_read_bool(np,"ktz,pwm-mode");
	LOG_DBG("pwmmode --<%d> \n", pdata->pwm_mode);

	pdata->using_lsb = of_property_read_bool(np, "ktz,using-lsb");
	LOG_DBG("using_lsb --<%d>\n", pdata->using_lsb);

	pdata->using_linear = of_property_read_bool(np, "ktz,using-linear");
	LOG_DBG("using_linear --<%d>\n", pdata->using_linear);


	if(pdata->using_lsb){
		pdata->default_brightness = 614;//614
		pdata->max_brightness = 2047;
	}else{
		pdata->default_brightness = 0x4D;//77
		pdata->max_brightness = 255;
	}

	rc = of_property_read_u32(np, "ktz,pwm-frequency", &temp);
	if(rc){
		pr_err("Invalid pwm-frequency!\n");
	}else{
		pdata->pwm_period = temp;
		LOG_DBG("pwm-frequency --<%d> \n", pdata->pwm_period);
	}
#ifdef CONFIG_HS_EPD_PANEL_SUPPORT	
	rc = of_property_read_u32(np, "ktz,brightness-scale", &temp);
	if(rc){
		pdata->brightness_scale = 0;
		pr_info("Invalid brightness-scale!\n");
	}else{
		pdata->brightness_scale = temp;
		printk("brightness-scale --<%d> \n", pdata->brightness_scale);
	}
#endif
		/*hisense add to limit minimal brightness*/
	rc = of_property_read_u32(np, "ktz,min-brightness-levle-limit", &temp);
	if(rc){
		pr_err("Invalid min-brightness-levle-limit! \n");
	}else{
		pdata->min_brightness_limit = temp;
		LOG_DBG("min-brightness-level-limit -- <%d> \n", pdata->min_brightness_limit);
	}/*hisense add end*/

	rc = of_property_read_u32(np, "ktz,bl-fscal-led", &temp);
	if(rc){
		pr_err("Invalid backlight full-scale led current!\n");
	}else{
		pdata->full_scale_led = temp;
		LOG_DBG("full-scale led current --<%d mA> \n", pdata->full_scale_led);
	}

	rc = of_property_read_u32(np, "ktz,turn-on-ramp", &temp);
	if(rc){
		pr_err("Invalid ramp timing ,,turnon!\n");
	}else{
		pdata->ramp_on_time = temp;
		LOG_DBG("ramp on time --<%d ms> \n", pdata->ramp_on_time);
	}

	rc = of_property_read_u32(np, "ktz,turn-off-ramp", &temp);
	if(rc){
		pr_err("Invalid ramp timing ,,turnoff!\n");
	}else{
		pdata->ramp_off_time = temp;
		LOG_DBG("ramp off time --<%d ms> \n", pdata->ramp_off_time);
	}

	rc = of_property_read_u32(np, "ktz,pwm-trans-dim", &temp);
	if(rc){
		pr_err("Invalid pwm-tarns-dim value!\n");
	}
	else{
		pdata->pwm_trans_dim = temp;
		LOG_DBG("pwm trnasition dimming  --<%d ms> \n", pdata->pwm_trans_dim);
	}

	rc = of_property_read_u32(np, "ktz,i2c-trans-dim", &temp);
	if(rc){
		pr_err("Invalid i2c-trans-dim value !\n");
	}else{
		pdata->i2c_trans_dim = temp;
		LOG_DBG("i2c transition dimming --<%d ms>\n", pdata->i2c_trans_dim);
	}

	rc = of_property_read_u32(np, "ktz,bl-channel", &bl_channel);
	if (rc) {
		pr_err("Invalid channel setup\n");
	}else{
		pdata->channel = bl_channel;
		LOG_DBG("bl-channel --<%x> \n", pdata->channel);
	}

	rc = of_property_read_u32(np, "ktz,ovp-level", &temp);
	if(!rc){
		pdata->ovp_level = temp;
		LOG_DBG("ovp-level --<%d> --temp <%d>\n", pdata->ovp_level, temp);
	}else
		pr_err("Invalid OVP level!\n");

	rc = of_property_read_u32(np, "ktz,switching-frequency", &temp);
	if(!rc){
		pdata->frequency = temp;
		LOG_DBG("switching frequency --<%d> \n", pdata->frequency);
	}else{
		pr_err("Invalid Frequency value!\n");
	}

	rc = of_property_read_u32(np, "ktz,inductor-current", &temp);
	if(!rc){
		pdata->induct_current = temp;
		LOG_DBG("inductor current limit --<%d> \n", pdata->induct_current);
	}else
		pr_err("invalid induct_current limit\n");

	rc = of_property_read_u32(np, "ktz,flash-timeout", &temp);
	if(!rc){
		pdata->flash_timeout = temp;
		LOG_DBG("flash timeout --<%d> \n", pdata->flash_timeout);
	}else{
		pr_err("invalid flash-time value!\n");
	}

	rc = of_property_read_u32(np, "ktz,flash-current", &temp);
	if(!rc){
		pdata->flash_current = temp;
		LOG_DBG("flash current --<0x%x> \n", pdata->flash_current);
	}else{
		pr_err("invalid flash current value!\n");
	}

	dev->platform_data = pdata;
	return;
}
/*
static void ktz8866_pwm_mode_enable(struct ktz8866_chip *chip, bool en)
{
	struct ktz8866_bl_pdata *pdata = chip->pdata;
	if(en){
		if(pdata->pwm_mode){
			LOG_DBG("already activated!\n");
			ktz8866_masked_write(chip->client, REG_PWM, 0x80, 0x00);
		}
	}else{
		if(pdata->pwm_mode){
			pdata->pwm_mode = en;
		}
		ktz8866_masked_write(chip->client, REG_PWM, 0x80, 0x80);
	}

	//ktz8866_read_reg(chip->client, REG_PWM, &value);
	//LOG_DBG("current value is --<%x>\n", value);
}
*/
static void ktz8866_ramp_setting(struct ktz8866_chip *chip)
{
	struct ktz8866_bl_pdata *pdata = chip->pdata;
	unsigned int max_time = 16384;
	int temp = 0;

	if(pdata->ramp_on_time == 0){//512us
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x40, 0x00);
		LOG_DBG("rampon time is 0 \n");
	}else if(pdata->ramp_on_time > max_time){
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x40, 0x40);
		LOG_DBG("rampon time is max \n");
	}else{
		temp = ktz_find_bit(pdata->ramp_on_time);
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x40, temp<<3);
		LOG_DBG("temp is %d\n", temp);
	}

	if(pdata->ramp_off_time == 0){//512us
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x04, 0x00);
		LOG_DBG("rampoff time is 0 \n");
	}else if(pdata->ramp_off_time > max_time){
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x04, 0x04);
		LOG_DBG("rampoff time is max \n");
	}else{
		temp = ktz_find_bit(pdata->ramp_off_time);
		ktz8866_masked_write(chip->client, REG_RAMP_ON, 0x04, temp);
		LOG_DBG("temp is %d\n", temp);
	}

}

static void ktz8866_transition_ramp(struct ktz8866_chip *chip)
{
	struct ktz8866_bl_pdata *pdata = chip->pdata;
	int reg_i2c, reg_pwm, temp;

	if(pdata->i2c_trans_dim >= 1024){
		reg_i2c = 0xf;
	}else if(pdata->i2c_trans_dim < 128){
		reg_i2c = 0x0;
	}else{
		temp =pdata->i2c_trans_dim/64;
		reg_i2c = temp-1;
		LOG_DBG("reg_i2c is --<0x%x>\n", reg_i2c);
	}

	if(pdata->pwm_trans_dim >= 256){
		reg_pwm = 0x7;
	}else if(pdata->pwm_trans_dim < 4){
		reg_pwm = 0x0;
	}else{
		temp = ktz_find_bit(pdata->pwm_trans_dim);
		reg_pwm = temp -2;
		LOG_DBG("temp is %d\n", temp);
	}
	printk(KERN_ERR "%s:[Hisense] reg_i2c=%d, reg_pwm=%d\n", __func__, reg_i2c, reg_pwm);
	//ktz8866_masked_write(chip->client, REG_TRANS_RAMP, 0x70, 0x70);
	ktz8866_masked_write(chip->client, REG_TRANS_RAMP, 0x07, 0x00);

}

static void ktz8866_flash_brightness_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct ktz8866_chip *chip;
	u8 reg;
	chip = container_of(cdev, struct ktz8866_chip, cdev_flash);

	cancel_delayed_work_sync(&chip->work);
	if(!brightness) // flash off
		return;
	else if(brightness > 15)
		brightness = 0x0f;

	if(chip->pdata->flash_timeout < 100){
		reg = 0x00;
	}else if(chip->pdata->flash_timeout > 1500){
		reg = 0x0f;
	}else{
		reg = (chip->pdata->flash_timeout/100);
	}

	reg = (reg << 4) | brightness;
	LOG_DBG("update register value --<0x%x>\n", reg);
	ktz8866_write_reg(chip->client, REG_FLASH_SETTING, reg);

	ktz8866_masked_write(chip->client, REG_MODE, 0x02, 0x02);

	schedule_delayed_work(&chip->work, chip->pdata->flash_timeout);
	return;
}

int ktz8866_flashled_init(struct i2c_client *client, struct ktz8866_chip *chip)
{
	//struct ktz8866_bl_pdata *pdata = chip->pdata;
	int ret;

	chip->cdev_flash.name = "ktz8866_flash";
	chip->cdev_flash.max_brightness = 16;
	chip->cdev_flash.brightness_set = ktz8866_flash_brightness_set;

	ret = led_classdev_register((struct device *) &client->dev, &chip->cdev_flash);
	if(ret<0){
		dev_err(&client->dev, "failed to register ktz8866_flash\n");
		return ret;
	}

	return 0;
}

static void ktz8866_backlight_init(struct ktz8866_chip *chip)
{
	struct ktz8866_bl_pdata *pdata = chip->pdata;
	u8 value = 0;
	u8 update_value = 0;

	update_value = (pdata->ovp_level == 32) ? 0x20 : 0x00;
	(pdata->induct_current == 2600) ? update_value |=0x08 : update_value;
	(pdata->frequency == 1000) ? update_value |=0x40: update_value;
	(pdata->using_linear == 1) ? update_value |=0x02: update_value;

	ktz8866_write_reg(chip->client, REG_BL_EN, 0x4F);
	ktz8866_masked_write(chip->client, REG_TRANS_RAMP, 0xF8, 0x98);
	ktz8866_write_reg(chip->client, REG_CONTROL, 0x8D);

	//if(pdata->pwm_mode)
		//ktz8866_pwm_mode_enable(chip, true);
	//else
		//ktz8866_pwm_mode_enable(chip, false);

	ktz8866_ramp_setting(chip);
	ktz8866_transition_ramp(chip);
	ktz8866_read_reg(chip->client, REG_CONTROL, &value);
	printk("[Hisense] ktz 3137 read control register -before--<0x%x> -after--<0x%x> \n",
					update_value, value);

	if (bl_force_flag) {
		pr_info("LCD need bl force for testing\n");
		return;
	}
	LOG_DBG("brightness=%d\n", brightness);
	//printk("%s: set brightness = %d", brightness);

	if ((last_brightness == 0) && (brightness != 0)) {
		if (ktz8866_global) {
			ktz8866_masked_write(ktz8866_global->client, REG_BL_EN,0x40,0x40);
	 	} else {
			LOG_DBG("ktz8866_reg_on ktz8866_global is null\n");
	 	}
	} else if (brightness == 0){
			if (ktz8866_global) {
				ktz8866_masked_write(ktz8866_global->client, REG_BL_EN,0x40,0x00);
		 	} else {
				LOG_DBG("ktz8866_reg_on ktz8866_global is null\n");
		 	}
			last_brightness = 0;
	}

 	if (ktz8866_global)
		ktz8866_brightness_set(ktz8866_global, brightness);
 	else
		LOG_DBG("ktz8866_brightness ktz8866_global is null\n");
	last_brightness = brightness;
}

EXPORT_SYMBOL(ktz8866_brightness);

static ssize_t ktz8866_bl_chip_id_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val= 0;

	ktz8866_read_reg(chip->client, REG_DEV_ID, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_mode_reg_show(struct device *dev, 
										struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_MODE, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_mode_reg_store(struct device *dev, 
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_MODE, value);

	return count;
}

static ssize_t ktz8866_bl_ctrl_reg_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_CONTROL, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_ctrl_reg_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_CONTROL, value);

	return count;

}

static ssize_t ktz8866_bl_brightness_reg_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint16_t reg_val = 0;
	u8 temp = 0;
#if 0
	if(chip->pdata->using_lsb){
		ktz8866_read_reg(chip->client, REG_RATIO_LSB, &temp);
		reg_val = temp << 8;
		ktz8866_read_reg(chip->client, REG_RATIO_MSB, &temp);
		reg_val |= temp;
	}else{
		ktz8866_read_reg(chip->client, REG_RATIO_MSB, &temp);
		reg_val = temp;
	}
#else
	ktz8866_read_reg(chip->client, REG_RATIO_LSB, &temp);
	reg_val = temp << 8;
	ktz8866_read_reg(chip->client, REG_RATIO_MSB, &temp);
	reg_val |= temp;
#endif 
	return sprintf(buf, "0x%x\n", reg_val);

}

static ssize_t ktz8866_bl_brightness_reg_store(struct device *dev,
									struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}


	if(chip->pdata->using_lsb){
			ktz8866_masked_write(chip->client, REG_RATIO_LSB, 0x07, value);
			ktz8866_masked_write(chip->client, REG_RATIO_MSB, 0xff, value >> 3);
		}else{
			ktz8866_write_reg(chip->client, REG_RATIO_MSB, value);	
		}

	return count;
}

static ssize_t ktz8866_bl_pwm_reg_show(struct device *dev, 
									struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_PWM, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_pwm_reg_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_PWM, value);

	return count;
}

static ssize_t ktz8866_bl_ramp_reg_show(struct device *dev, 
									struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_RAMP_ON, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_ramp_reg_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_RAMP_ON, value);

	return count;

}

static ssize_t ktz8866_bl_trans_ramp_reg_show(struct device *dev, 
									struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_TRANS_RAMP, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_trans_ramp_reg_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_TRANS_RAMP, value);

	return count;

}

static ssize_t ktz8866_bl_flash_setting_reg_show(struct device *dev, 
									struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_FLASH_SETTING, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_flash_setting_reg_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	ktz8866_write_reg(chip->client, REG_FLASH_SETTING, value);

	return count;
}

static ssize_t ktz8866_bl_status_reg_show(struct device *dev, 
									struct device_attribute *attr, char *buf)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val = 0;

	ktz8866_read_reg(chip->client, REG_STATUS, &reg_val);
	return sprintf(buf, "0x%x\n", reg_val);
}

static ssize_t ktz8866_bl_force_show(struct device *dev, 
										struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bl_force_flag);
}

static ssize_t ktz8866_bl_force_store(struct device *dev, 
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktz8866_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if(ret){
		dev_err(chip->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	bl_force_flag = value;

	return count;
}
static DEVICE_ATTR(bl_force, 0664, ktz8866_bl_force_show, ktz8866_bl_force_store);

static DEVICE_ATTR(ktz_chip_id, 0444, ktz8866_bl_chip_id_show, NULL);
static DEVICE_ATTR(ktz_mode_reg, 0664, ktz8866_bl_mode_reg_show, ktz8866_bl_mode_reg_store);
static DEVICE_ATTR(ktz_ctrl_reg, 0664, ktz8866_bl_ctrl_reg_show, ktz8866_bl_ctrl_reg_store);
static DEVICE_ATTR(ktz_brightness_reg, 0664, ktz8866_bl_brightness_reg_show, 
				ktz8866_bl_brightness_reg_store);
static DEVICE_ATTR(ktz_pwm_reg, 0664, ktz8866_bl_pwm_reg_show, ktz8866_bl_pwm_reg_store);
static DEVICE_ATTR(ktz_ramp_reg, 0664, ktz8866_bl_ramp_reg_show, ktz8866_bl_ramp_reg_store);
static DEVICE_ATTR(ktz_trans_ramp_reg, 0664, ktz8866_bl_trans_ramp_reg_show, 
				ktz8866_bl_trans_ramp_reg_store);
static DEVICE_ATTR(ktz_flash_setting_reg, 0664, ktz8866_bl_flash_setting_reg_show, 
				ktz8866_bl_flash_setting_reg_store);
static DEVICE_ATTR(ktz_status_reg, 0444, ktz8866_bl_status_reg_show, NULL);

static struct attribute *ktz8866_bl_attribute[] = {
	&dev_attr_ktz_chip_id.attr,
	&dev_attr_bl_force.attr,
	&dev_attr_ktz_mode_reg.attr,
	&dev_attr_ktz_ctrl_reg.attr,
	&dev_attr_ktz_brightness_reg.attr,
	&dev_attr_ktz_pwm_reg.attr,
	&dev_attr_ktz_ramp_reg.attr,
	&dev_attr_ktz_trans_ramp_reg.attr,
	&dev_attr_ktz_flash_setting_reg.attr,
	&dev_attr_ktz_status_reg.attr,
	NULL
};

static const struct attribute_group ktz8866_bl_attr_group = {
	.attrs = ktz8866_bl_attribute,
};

static int ktz8866_probe(struct i2c_client *client, const struct i2c_device_id *id){
	int ret;
	int err = 0;
	//struct backlight_device *bl;
	struct ktz8866_bl_pdata *pdata = dev_get_drvdata(&client->dev);
	struct ktz8866_chip *chip;
	//struct device_node *np = client->dev.of_node;

	printk("%s:probe start!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if(!chip){
		dev_err(&client->dev, "%s: failed to allocate driver data.\n", __func__);
		err = -ENOMEM;
		goto exit0;
	}

	if(!pdata){
		ktz_parse_dt(&client->dev, chip);
		pdata = dev_get_platdata(&client->dev);
	}
	chip->client = client;
	chip->pdata = pdata;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	ktz8866_client = client;

#ifdef ktz8866_USE_PINCTRL
	ktz8866_pinctrl_init(chip);
	ktz8866_pinctrl_select(pdata,true);
#endif
	ret = gpio_request(pdata->hwen_gpio, "ktz_hwen_gpio");
	if(ret<0){
		pr_err("ktz8866 failed to request gpio\n");
		return -1;
	}

	ret = gpio_direction_output(pdata->hwen_gpio, 1);

	ktz8866_backlight_init(chip);
	/*
	INIT_DELAYED_WORK(&chip->work, ktz8866_sync_backlight_work);
	ktz8866_flashled_init(client, chip);
	ktz8866_check_status(chip);

	backlight_update_status(chip->bl);
	*/
	ret = sysfs_create_group(&client->dev.kobj, &ktz8866_bl_attr_group);
	if (ret) {
		pr_err("failed to create sysfs device attributes\n");
	}
	ktz8866_global = chip;
#ifdef CONFIG_HS_EPD_PANEL_SUPPORT
	/**
	same to uboot backlight level--6
	brightness = (brightness - 1) * (pdata->brightness_scale - 1) / (pdata->max_brightness - 1) + 1;
	color:16      brightness = 15*767/2046+1 = 6
	whiteblack:56 brightness = 55*203/2046+1 = 6
	*/
	if(!chip->pdata->brightness_scale)//clolr
		ktz8866_brightness(24);// 255 -3
	else//wb
		ktz8866_brightness(56);// 255 -7
#else
	ktz8866_brightness(614); //2047*30%
#endif
	printk("%s:probe end!\n", __func__);
exit0:
	return err;
}

static int ktz8866_remove(struct i2c_client *client)
{
	//struct ktz8866_chip *chip = i2c_get_clientdata(client);

	//chip->bl->props.brightness = 0;

	//backlight_update_status(chip->bl);
	//cancel_delayed_work_sync(&chip->work);

	sysfs_remove_group(&client->dev.kobj, &ktz8866_bl_attr_group);
	//gpio_free(chip->pdata->hwen_gpio);

	return 0;
}

static const struct i2c_device_id ktz8866_id[] = {
	{KTZ_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktz8866_match_table[] = {
        { .compatible = "ktz,ktz8866",},
        { },
};

static struct i2c_driver ktz8866_driver = {
	.driver = {
		.name	= KTZ_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktz8866_match_table,
	},
	.probe = ktz8866_probe,
	.remove = ktz8866_remove,
	.id_table = ktz8866_id,
};

static int __init ktz8866_init(void)
{
	int err;
	err = i2c_add_driver(&ktz8866_driver);
	if (err) {
		printk(KERN_WARNING "ktz8866 driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktz8866_driver.driver.name);
	}
	return err;
}

static void __exit ktz8866_exit(void)
{
	i2c_del_driver(&ktz8866_driver);
}

module_init(ktz8866_init);
module_exit(ktz8866_exit);

MODULE_AUTHOR("kinet-ic.com");
MODULE_LICENSE("GPL");

