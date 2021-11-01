// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Sockets/Public/IPAddress.h"
#include "Common/UdpSocketBuilder.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketSender.h"
#include "Containers/List.h"
#include "OpenSeeComponent.generated.h"

USTRUCT(BlueprintType)
struct OPENSEETRACKING_API FOpenSeeFeatureData
{
	GENERATED_USTRUCT_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyeLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyeRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowSteepnessLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowUpDownLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowQuirkLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowSteepnessRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowUpDownRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float EyebrowQuirkRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthCornerUpDownLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthCornerInOutLeft = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthCornerUpDownRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthCornerInOutRight = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthOpen = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float MouthWide = 0;
};

USTRUCT(BlueprintType)
struct OPENSEETRACKING_API FOpenSeeTrackingData
{
	GENERATED_USTRUCT_BODY()
public:
	int nPoints = 68;
	double time;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		int32 id = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FVector2D cameraResolution;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float rightEyeOpen = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float leftEyeOpen = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FQuat rightGaze;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FQuat leftGaze;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		bool got3DPoints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		float fit3DError = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FVector rotation = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FVector translation = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FQuat rawQuaternion;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FVector rawEuler;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		TArray<float> confidence;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		TArray<FVector2D> points;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		TArray<FVector> points3D;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FOpenSeeFeatureData features;

	FQuat LookRotation(FVector lookAt, FVector upDirection);
	float readFloat(uint8_t* bytes, int& offset);
	double readDouble(uint8_t* bytes, int& offset);
	int readInt(uint8_t* bytes, int& offset);
	FVector2D readVector2(uint8_t* bytes, int& offset);
	FVector readVector3(uint8_t* bytes, int& offset);
	FVector swapX(FVector v);
	FQuat readQuaternion(uint8_t* bytes, int& offset);
	void readFromPacket(uint8_t* b, int o);
};

USTRUCT(BlueprintType)
struct OPENSEETRACKING_API FUDPNetworkSettings
{
	GENERATED_USTRUCT_BODY()

	/** Default sending socket IP string in form e.g. 127.0.0.1. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
	FString SendIP;

	/** Default connection port e.g. 3001*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		int32 SendPort;

	/** Port to which send is bound to on this client (this should change on each open) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		int32 SendBoundPort;

	/** Default receiving socket IP string in form e.g. 0.0.0.0 for all connections, may need 127.0.0.1 for some cases. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FString ReceiveIP;

	/** Default connection port e.g. 3002*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		int32 ReceivePort;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FString SendSocketName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FString ReceiveSocketName;

	/** in bytes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		int32 BufferSize;

	/** If true will auto-connect on begin play to IP/port specified for sending udp messages, plus when emit is called */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		bool bShouldAutoOpenSend;

	/** If true will auto-listen on begin play to port specified for receiving udp messages. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		bool bShouldAutoOpenReceive;

	/** This will open it to the bound send port at specified send ip and ignore passed in settings for open receive. Default False*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		bool bShouldOpenReceiveToBoundSendPort;

	/** Whether we should process our data on the gamethread or the udp thread. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		bool bReceiveDataOnGameThread;

	UPROPERTY(BlueprintReadOnly, Category = "OpenSeeUE")
		bool bIsReceiveOpen;

	UPROPERTY(BlueprintReadOnly, Category = "OpenSeeUE")
		bool bIsSendOpen;

	FUDPNetworkSettings();
};

class OPENSEETRACKING_API FUDPMessenger
{
public:

	TFunction<void(FOpenSeeTrackingData, const FString&)> OnReceivedData;
	TFunction<void(int32 Port)> OnReceiveOpened;
	TFunction<void(int32 Port)> OnReceiveClosed;
	TFunction<void(int32 SpecifiedPort, int32 BoundPort)> OnSendOpened;
	TFunction<void(int32 Port)> OnSendClosed;

	FUDPNetworkSettings Settings;

	FUDPMessenger();
	~FUDPMessenger();

	int32 OpenSendSocket(const FString& InIP = TEXT("127.0.0.1"), const int32 InPort = 3000);
	bool CloseSendSocket();

	bool EmitBytes(const TArray<uint8>& Bytes);

	bool OpenReceiveSocket(const FString& InIP = TEXT("127.0.0.1"), const int32 InListenPort = 11573);
	bool CloseReceiveSocket();

	void ClearSendCallbacks();
	void ClearReceiveCallbacks();

protected:
	FSocket* SenderSocket;
	FSocket* ReceiverSocket;
	FUdpSocketReceiver* UDPReceiver;
	FString SocketDescription;
	TSharedPtr<FInternetAddr> RemoteAdress;
	ISocketSubsystem* SocketSubsystem;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUDPSocketStateSig, int32, Port);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FUDPSocketSendStateSig, int32, SpecifiedPort, int32, BoundPort);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FUDPMessageSig, FOpenSeeTrackingData, SeeData, const FString&, IPAddress);

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class OPENSEETRACKING_API UOpenSeeComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UOpenSeeComponent();

	UPROPERTY(BlueprintAssignable, Category = "OpenSeeUE")
		FUDPMessageSig OnReceivedBytes;

	UPROPERTY(BlueprintAssignable, Category = "OpenSeeUE")
		FUDPSocketStateSig OnReceiveSocketOpened;

	UPROPERTY(BlueprintAssignable, Category = "OpenSeeUE")
		FUDPSocketStateSig OnReceiveSocketClosed;

	UPROPERTY(BlueprintAssignable, Category = "OpenSeeUE")
		FUDPSocketSendStateSig OnSendSocketOpened;

	UPROPERTY(BlueprintAssignable, Category = "OpenSeeUE")
		FUDPSocketStateSig OnSendSocketClosed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OpenSeeUE")
		FUDPNetworkSettings Settings;

	UFUNCTION(BlueprintCallable, Category = "OpenSeeUE")
		int32 OpenSendSocket(const FString& InIP = TEXT("127.0.0.1"), const int32 InPort = 3000);

	UFUNCTION(BlueprintCallable, Category = "OpenSeeUE")
		bool CloseSendSocket();

	UFUNCTION(BlueprintCallable, Category = "OpenSeeUE")
		bool OpenReceiveSocket(const FString& InListenIP = TEXT("127.0.0.1"), const int32 InListenPort = 11573);

	UFUNCTION(BlueprintCallable, Category = "OpenSeeUE")
		bool CloseReceiveSocket();

	UFUNCTION(BlueprintCallable, Category = "OpenSeeUE")
		bool EmitBytes(const TArray<uint8>& Bytes);

protected:
	TSharedPtr<FUDPMessenger> Native;

	void LinkupCallbacks();
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
};
