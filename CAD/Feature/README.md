# 一、孔识别
以下钻孔类型在工业中最常见：

1. 通孔。
2. 沉头孔。
3. 埋头孔。
4. 盲孔。

| 类型 | 图例 |
| --- | --- |
| 通孔 | <img src="../../PIC/1780888374051-2a7d3ddb-ca45-45c0-b047-576bf047ba79.png" width="112.33522033691406" title="" crop="0,0,1,1" id="u234566cc" class="ne-image"> |
| 沉头孔 | <img src="../../PIC/1780888453716-86392104-44f1-430b-964c-72dd8626f1bb.png" width="111.33522033691406" title="" crop="0,0,1,1" id="u638c4652" class="ne-image"> |
| 埋头孔 | <img src="../../PIC/1780888502130-f2acfa99-6cc7-44dc-ad8e-14796f0b92af.png" width="111.33522033691406" title="" crop="0,0,1,1" id="u27ee0939" class="ne-image"> |
| 盲孔 | <img src="../../PIC/1780888566283-ddcf39f1-e680-43ff-9124-e9acb49e98a7.png" width="113.33522033691406" title="" crop="0,0,1,1" id="u4e1c6170" class="ne-image"> |


识别这些类型对制造规划、成本估算、特征去除以及其他采用CAD几何数据作为数据源的计算流程至关重要。

本算法识别由圆柱，圆锥，平面组成的孔，对于更复杂的孔识别，可以参见腔体识别

## 使用方法
```cpp
#include <FR_RecognizeDrillHoles.h>

TopoDS_Shape shape = ...;

FR_RecognizeDrillHoles recognizer(shape);

if ( recognizer.Perform(50.0) )//传入最大半径
{
    // Get all recognized hole face IDs.
    const FR_Feature& resIndices = recognizer.GetResultIndices();

    //get TopoDS_Face
    for (TColStd_PackedMapOfInteger::Iterator it(resIndices); it.More(); it.Next())
    {
        TopoDS_Face f = recognizer.GetAAG()->GetFace(it.Key());
        //todo somthing
    }

}
```

# 二、轴识别
识别轴的加工件，是孔洞的逆运算

<img src="../../PIC/1780890736702-cebc3460-057f-4e1d-b890-675cb0e79c10.png" width="263.63635792219947" title="" crop="0,0,1,1" id="u94ae4692" class="ne-image">



## 使用方法
```cpp
#include <FR_RecognizeShafts.h>
#include <FR_AAG.h>

TopoDS_Shape shape = ...;

FR_RecognizeShafts recognizer(new FR_AAG(shape));

if ( recognizer.Perform(50.0) )//传入最大半径
{
    // Get all recognized shaft face IDs.
    const FR_Feature& resIndices = recognizer.GetResultIndices();

    //get TopoDS_Face
    for (TColStd_PackedMapOfInteger::Iterator it(resIndices); it.More(); it.Next())
    {
        TopoDS_Face f = recognizer.GetAAG()->GetFace(it.Key());
        //todo somthing
    }

}
```



# 三、圆角识别
圆角是CAD模型中的一个特征，用于通过用过渡（圆角）面替换两个面的交线从而从零件中删除尖边。圆角通常用作模型上的精加工特征，在编辑零件几何图形时，希望保留其语义。

目前支持的圆角识别功能：

| 功能 | 图例 |
| --- | --- |
| 圆柱面圆角识别 | <img src="../../PIC/1780897893149-f1432e6a-63af-4ed6-84db-866b0a3c1900.png" width="221.272705078125" title="" crop="0,0,1,1" id="u897aa60b" class="ne-image"> |
| 圆环面圆角识别 | <img src="../../PIC/1780897931211-6eb379d5-61ec-4de8-a731-cab39b0927b3.png" width="232.90908813476562" title="" crop="0,0,1,1" id="u6f0fa916" class="ne-image"> |
| 样条面圆角识别 | <img src="../../PIC/1780897958787-93a74f4f-b232-41e1-ac60-e89fdd688851.png" width="247.45452880859375" title="" crop="0,0,1,1" id="u0cf4fc5f" class="ne-image"> |
| 圆锥面圆角识别 | <img src="../../PIC/1780898025534-085206bf-c433-4215-b2b3-6f4f1f03420b.png" width="255.3238525390625" title="" crop="0,0,1,1" id="u8ad0595c" class="ne-image"> |
| 边面过渡圆角识别 | <img src="../../PIC/1780898086493-d6978121-3bbf-476d-8800-92b0b061b8be.png" width="189.36361694335938" title="" crop="0,0,1,1" id="uf29c9372" class="ne-image"> |
| 球面顶点圆角识别 | <img src="../../PIC/1780898124783-fdd65394-d611-4751-ad9c-69a2adf948a3.png" width="254.3238525390625" title="" crop="0,0,1,1" id="ufbd31360" class="ne-image"> |
| 圆角链提取 | 圆柱面，球面混合圆角链<br/><img src="../../PIC/1777013693699-3c59f245-561b-4a2c-ba67-397b020e61cd.png" width="300.3238525390625" title="" crop="0,0,1,1" id="OY7oA" class="ne-image"><br/>圆角链识别结果<br/><img src="../../PIC/1777013754673-7ee5d379-25d5-4475-953a-dcf28bf365ca.png" width="93.31533813476562" title="" crop="0,0,1,1" id="DUcJE" class="ne-image"><img src="../../PIC/1777013763342-998c9ccc-05f9-4c5f-b616-3d6eca1da4a4.png" width="107.31533813476562" title="" crop="0,0,1,1" id="Ez5QI" class="ne-image"><img src="../../PIC/1777013772483-b3a2fdcc-8c1b-45ad-b47f-4ec7619b0b7d.png" width="166.30397033691406" title="" crop="0,0,1,1" id="wqnfZ" class="ne-image"> |
| | 圆柱面，圆环面混合圆角链<br/><img src="../../PIC/1777017418154-ee10a9f9-8ba2-47d4-97f2-f29eb359bee3.png" width="214.3238525390625" title="" crop="0,0,1,1" id="BwT8I" class="ne-image"><br/>圆角链识别结果<br/><img src="../../PIC/1777017460611-f1786c14-bf70-42c0-94da-de0e9a49314e.png" width="83.99147033691406" title="" crop="0,0,1,1" id="DIUk4" class="ne-image"><img src="../../PIC/1777017470828-d7f347ef-e471-4188-a193-90ba54deb121.png" width="330.3011169433594" title="" crop="0,0,1,1" id="yM1Kt" class="ne-image"> |
| 圆角链顺序确定 | <img src="../../PIC/1777018296574-bed63785-fd98-4319-9a66-d7079a724be0.png" width="184.3238525390625" title="" crop="0,0,1,1" id="Y23M2" class="ne-image"><br/>圆角链顺序<br/><img src="../../PIC/1777018340124-75bac1bb-75e2-493f-9233-5e4fdc761567.png" width="164.3238525390625" title="" crop="0,0,1,1" id="Sxxt7" class="ne-image"><img src="../../PIC/1777018349367-8dcd806b-c3f2-4776-bc33-d9c66854c9af.png" width="260.3238525390625" title="" crop="0,0,1,1" id="Hc1Jm" class="ne-image"> |
| | <img src="../../PIC/1777270974055-b3a98987-568b-4185-b4a4-00cdbcf5cc70.png" width="248.3238525390625" title="" crop="0,0,1,1" id="TxRXS" class="ne-image"><br/>圆角链顺序<br/><img src="../../PIC/1777271037672-9e7ec2c8-e64b-4206-bb92-cb86e190944f.png" width="187.31533813476562" title="" crop="0,0,1,1" id="Wqxva" class="ne-image"><img src="../../PIC/1777271065451-ec65e896-cf7d-491f-aa7e-75adf8d681c5.png" width="73.99147033691406" title="" crop="0,0,1,1" id="uuXcL" class="ne-image"><img src="../../PIC/1777271151306-5d2427a3-ffac-436b-874f-4ea20bb3a357.png" width="198.30397033691406" title="" crop="0,0,1,1" id="JR1zy" class="ne-image"><br/><img src="../../PIC/1777271094025-7610508c-6fa8-4b52-9051-982e7ea0f36d.png" width="90.98579406738281" title="" crop="0,0,1,1" id="HITeP" class="ne-image"><img src="../../PIC/1777271122645-aee3245f-0113-4409-b5a3-46c4b1cc7300.png" width="225.3238525390625" title="" crop="0,0,1,1" id="Fcjmr" class="ne-image"> |


## 使用方法
```cpp
#include <BlendRecognizer.h>
#include <AttributeAdjacentGraph.h>

TopoDS_Shape shape = ...;

BlendRecognizer recognizer(shape);
//recognize all blends
auto blends = recognizer.RecognizeBlends();

for (CskIndexedDataMapOfShapeBlendProperty::Iterator it(blends); it.More(); it.Next())
{
    std::shared_ptr<BlendProperty> blendInfo = it.Value();
    double angle = blendInfo->GetAngle();
    double radius = blendInfo->GetRadius();
    blendInfo->GetSpringEdgeIndices();

    recognizer.GetAag();

    //todo something
}

//extract blend chains
std::vector<std::vector<TopoDS_Face>> chains = recognizer.ExtractBlendChains();

//sort blend chains according the generating order
std::vector<std::vector<TopoDS_Face>> orderedChains = recognizer.ExtractOrderedBlendChains();
```

# 四、倒角识别
倒角是CAD模型中的一个特征，用于通过用倾斜（倒角）面替换两个面的交线从零件中删除尖边。倒角通常用作模型上的精加工特征，在编辑零件几何图形时，希望保留其语义。

目前支持的倒角识别功能：

| 功能 | 图例 |
| --- | --- |
| 平面倒角识别 | <img src="../../PIC/1779155254184-ed108480-a799-47bc-bcd8-1fd22ede63f1.png" width="221.34371948242188" title="" crop="0,0,1,1" id="YnC8K" class="ne-image"> |
| 锥面倒角识别 | <img src="../../PIC/1779160951927-4fadd358-f0a5-4bcf-b939-73890c949d87.png" width="215.34371948242188" title="" crop="0,0,1,1" id="tNKbN" class="ne-image"> |
| 孤立顶点倒角识别 | <img src="../../PIC/1779176182909-30cf750e-2aa0-4223-a0a6-3edbe8e43129.png" width="215.34371948242188" title="" crop="0,0,1,1" id="VJhl1" class="ne-image"> |
| 连接顶点倒角识别 | <img src="../../PIC/1780900878231-0fbfa392-deff-4015-941f-7c41a3c16559.png" width="232.34371948242188" title="" crop="0,0,1,1" id="u7fc44586" class="ne-image"> |
| 倒角链提取 | <img src="../../PIC/1779259594732-cc45926c-08ca-4cd6-b6c0-86771256b20a.png" width="213.34371948242188" title="" crop="0,0,1,1" id="poYPm" class="ne-image"> |
| | <img src="../../PIC/1779266795652-efa885eb-e387-4eb5-810b-0ffcd35d11eb.png" width="205.34371948242188" title="" crop="0,0,1,1" id="QiPQY" class="ne-image"><br/>倒角链提取结果<br/><img src="../../PIC/1779329454342-1c269df4-6a07-4986-aaa1-60e3f7d7bf22.png" width="191.34371948242188" title="" crop="0,0,1,1" id="yvdpk" class="ne-image"><img src="../../PIC/1779329485249-e675a11b-ea94-4b89-a323-0657c950992a.png" width="191.34371948242188" title="" crop="0,0,1,1" id="Da77W" class="ne-image"><img src="../../PIC/1779329568834-d8053816-8219-432f-810c-d383e14d2592.png" width="191.99429321289062" title="" crop="0,0,1,1" id="Ad5Iu" class="ne-image"><img src="../../PIC/1779329624557-c783c8f8-a86e-4301-9c80-6f99b2d42e68.png" width="188.34371948242188" title="" crop="0,0,1,1" id="kSI8I" class="ne-image"><img src="../../PIC/1779329789364-c5c0098c-d14c-409d-aee4-929db7cfa80e.png" width="186.34371948242188" title="" crop="0,0,1,1" id="k6qEe" class="ne-image"><img src="../../PIC/1779329841433-8aee6435-3671-4345-a03d-026e4f2c160d.png" width="186.99713134765625" title="" crop="0,0,1,1" id="tZwgg" class="ne-image"><img src="../../PIC/1779329887709-8f36e312-cba5-42d6-b1b2-5f436016e002.png" width="195.34371948242188" title="" crop="0,0,1,1" id="spHE4" class="ne-image"> |


## 使用方法
```cpp
#include <ChamferRecognizer.h>
#include <AttributeAdjacentGraph.h>

TopoDS_Shape shape = ...;

ChamferRecognizer recognizer(shape);
//recognize all chamfers

const std::vector<ChamferFeature> features = recognizer.RecognizeChamfers();

for (const ChamferFeature& feature : features)
{
    feature.beltWidth;
    feature.faceId;
    feature.face;
    feature.supportFaces;
    //todo something
}

// extract chamfer chains
std::vector<std::vector<TopoDS_Face>> chains = recognizer.ExtractChamferChains();

// sort chamfer chains according the generating order
std::vector<std::vector<TopoDS_Face>> orderedChains = recognizer.ExtractOrderedBlendChains();

```

# 五、基面识别
基面是指有可能有特征（一般是孔，腔体）附着的面



<img src="../../PIC/1780384720832-32ab9159-d8ef-4111-a18c-ed38662fc969.png" width="490.178955078125" title="" crop="0,0,1,1" id="ywbwT" class="ne-image">

## 使用方法
```cpp
#include <FR_RecognizeBaseFaces.h>
#include <AttributeAdjacentGraph.h>

TopoDS_Shape shape = ...;

FR_RecognizeBaseFaces recognizer(new FR_AAG(shape));

if(recognizer.Perform())
{
    // Get all recognized base face IDs.
    const FR_Feature& baseFaceIds = recognizer.GetResultIndices();

    for (FR_Feature::Iterator it(baseFaceIds); it.More(); it.Next())
    {
        const CskTopoDS_Face f = recognizer.GetAAG()->GetFace(it.Key());
        //todo something
    }
}

```



# 六、腔体识别
腔体特征的识别允许检测所有加工体积，而无需区分其类型。根据我们的经验，这种识别器在以下情况下可能很有用：

1. 模型简化（抑制特征）。
2. 下游识别。在搜索某些特征时，可以将腔体特征排除在外。例如，在检测车削（车床）零件的轴线时，可以（也应该）忽略腔体的贡献。

<img src="../../PIC/1780905642683-53b7fb58-ea11-441c-a620-7780f249a74f.png" width="356.178955078125" title="" crop="0,0,1,1" id="u49f881b9" class="ne-image"><img src="../../PIC/1780905664722-12135b2b-849a-4145-ae8a-baa5f4d18dc6.png" width="354.178955078125" title="" crop="0,0,1,1" id="ub61da628" class="ne-image"><img src="../../PIC/1780905688625-63bd6cca-ad33-4244-87fe-a6d13069cd18.png" width="507.178955078125" title="" crop="0,0,1,1" id="u518d0d3e" class="ne-image">

## 使用方法
```cpp
#include <FR_RecognizeCavities.h>

TopoDS_Shape shape = ...;

FR_RecognizeCavities recognizer(shape);
recognizer.SetMaxSize(50.0); // 设置过渡腔体包围核大小

if ( recognizer.Perform() )
{
  // Get all recognized cavity face IDs.
  const FR_Feature& cavityIds = recognizer.GetResultIndices();

  // Get cavities with their base faces.
  const FR_RecognizeCavities::t_cavities& cavities = recognizer.GetCavities();

  for ( const auto& cavity : cavities )
  {
    const FR_Feature& featureFaces = cavity.first;
    const FR_Feature& baseFaces    = cavity.second;
    // to do something
  }
}

```

# 八、依赖环境
+ <font style="color:rgb(31, 35, 40);">C++17</font>
+ <font style="color:rgb(31, 35, 40);">OCCT（OpenCASCADE >= 7.2.0）</font>
+ Eigen-3.4.0
